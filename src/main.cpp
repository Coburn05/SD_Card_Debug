// SD card over SPI1 debug test for the Nucleo-H753ZI.
//
// Wiring (SPI1):
//   CS   -> PB6  (driven manually as GPIO)
//   SCK  -> PA5  (Nucleo CN7 D13)
//   MISO -> PA6  (Nucleo CN7 D12)
//   MOSI -> PB5  (Nucleo CN7 D22)  -- PA7 is used by Ethernet on this board
//
// Stage 1 talks to the card with raw SD SPI-mode commands so every response
// byte is visible. That tells you whether the wiring/power/card is the problem.
// Stage 2 mounts the card with SdFat and does a file write/read-back/speed test.
//
// Output goes to Serial (ST-Link virtual COM port), 115200 baud.

#include <Arduino.h>
#include <ArduinoLog.h>
#include <SPI.h>
#include <SdFat.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
static constexpr uint32_t SD_CS = PB6;
static constexpr uint32_t SD_SCK = PA5;
static constexpr uint32_t SD_MISO = PA6;
static constexpr uint32_t SD_MOSI = PB5;

static constexpr uint32_t INIT_CLOCK_HZ = 250000;  // SD spec: <= 400 kHz during init
static constexpr uint32_t FAST_CLOCK_MHZ = 16;      // for the SdFat stage; lower if flaky wiring

static constexpr char TEST_FILE[] = "/spi_test.bin";
static constexpr size_t TEST_BLOCK_SIZE = 4096;
static constexpr size_t TEST_BLOCK_COUNT = 256;  // 1 MiB total

// Speed sweep: tries every SCK the SPI1 prescaler can produce (kernel / 2^n), slowest
// first, and stops at the first speed where mount, write or read-back verify fails.
static constexpr uint32_t SWEEP_MIN_HZ = 1000000;  // skip steps below this
static constexpr size_t SWEEP_BLOCK_COUNT = 128;   // 512 KiB per pass
static constexpr int SWEEP_PASSES = 3;             // every pass must succeed to count
static constexpr char SWEEP_FILE[] = "/SWEEP.BIN";

// Human-readable file for checking the card on another device (PC, phone...).
// One entry is appended per run, so it also shows that earlier writes survived.
static constexpr char READABLE_FILE[] = "/SPITEST.TXT";
static constexpr char CHECK_LINE[] = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 0123456789";

// Pins not on the hardware SS line, so CS is managed by hand (ssel left as NC).
SPIClass sdSpi(SD_MOSI, SD_MISO, SD_SCK);
SdFs sd;
bool sdMounted = false;
uint32_t cardSerial = 0;  // PSN from the CID, filled in by stage 1
int runNumber = 0;        // entry number in READABLE_FILE, set by stage 3

// ---------------------------------------------------------------------------
// Raw SD SPI-mode helpers
// ---------------------------------------------------------------------------
static const SPISettings initSettings(INIT_CLOCK_HZ, MSBFIRST, SPI_MODE0);

static inline void csLow() { digitalWrite(SD_CS, LOW); }
static inline void csHigh() {
  digitalWrite(SD_CS, HIGH);
  sdSpi.transfer(0xFF);  // extra clocks so the card releases MISO
}

static uint8_t waitReady(uint32_t timeoutMs) {
  uint32_t start = millis();
  uint8_t b;
  do {
    b = sdSpi.transfer(0xFF);
    if (b == 0xFF) return b;
  } while (millis() - start < timeoutMs);
  return b;
}

// Sends a command and returns the R1 response (0xFF = no response).
// Leaves CS low so the caller can read any trailing bytes; caller must csHigh().
static uint8_t sendCmd(uint8_t cmd, uint32_t arg) {
  if (cmd & 0x80) {  // ACMD: prefix with CMD55
    cmd &= 0x7F;
    uint8_t r = sendCmd(55, 0);
    csHigh();
    if (r > 1) return r;
  }

  csLow();
  if (cmd != 0) waitReady(500);

  uint8_t crc = 0x01;
  if (cmd == 0) crc = 0x95;  // valid CRC for CMD0(0)
  if (cmd == 8) crc = 0x87;  // valid CRC for CMD8(0x1AA)

  sdSpi.transfer(0x40 | cmd);
  sdSpi.transfer(arg >> 24);
  sdSpi.transfer(arg >> 16);
  sdSpi.transfer(arg >> 8);
  sdSpi.transfer(arg);
  sdSpi.transfer(crc);

  if (cmd == 12) sdSpi.transfer(0xFF);  // skip stuff byte for STOP_TRANSMISSION

  uint8_t r1 = 0xFF;
  for (int i = 0; i < 10; i++) {
    r1 = sdSpi.transfer(0xFF);
    if (!(r1 & 0x80)) break;
  }
  return r1;
}
#define ACMD(n) (0x80 | (n))

// Reads a data block following a command (CSD/CID/sector). Returns true on success.
static bool readDataBlock(uint8_t *buf, size_t len) {
  uint32_t start = millis();
  uint8_t token;
  do {
    token = sdSpi.transfer(0xFF);
  } while (token == 0xFF && millis() - start < 300);

  if (token != 0xFE) {
    Log.errorln("  bad data token: 0x%x (expected 0xFE)", token);
    return false;
  }
  for (size_t i = 0; i < len; i++) buf[i] = sdSpi.transfer(0xFF);
  sdSpi.transfer(0xFF);  // CRC (ignored)
  sdSpi.transfer(0xFF);
  return true;
}

static void hexDump(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (i % 16 == 0) Serial.printf("    %04x: ", (unsigned)i);
    Serial.printf("%02X ", buf[i]);
    if (i % 16 == 15 || i == len - 1) Serial.println();
  }
}

static void explainR1(uint8_t r1) {
  if (r1 == 0xFF) {
    Log.errorln("  R1=0xFF: no response. MISO stuck high / floating, card not powered,");
    Log.errorln("           CS not reaching card, or SCK/MOSI not connected.");
    return;
  }
  if (r1 & 0x01) Log.noticeln("  R1 bit0: in idle state");
  if (r1 & 0x02) Log.warningln("  R1 bit1: erase reset");
  if (r1 & 0x04) Log.warningln("  R1 bit2: illegal command");
  if (r1 & 0x08) Log.warningln("  R1 bit3: CRC error");
  if (r1 & 0x10) Log.warningln("  R1 bit4: erase sequence error");
  if (r1 & 0x20) Log.warningln("  R1 bit5: address error");
  if (r1 & 0x40) Log.warningln("  R1 bit6: parameter error");
}

// ---------------------------------------------------------------------------
// Stage 1: raw protocol test
// ---------------------------------------------------------------------------
static bool rawProtocolTest() {
  Log.infoln("===== Stage 1: raw SD SPI protocol @ %l Hz =====", INIT_CLOCK_HZ);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  sdSpi.begin();
  sdSpi.beginTransaction(initSettings);

  // >= 74 clocks with CS and MOSI high to put the card in native mode
  for (int i = 0; i < 10; i++) sdSpi.transfer(0xFF);

  // Quick bus sanity check: with CS high, a card should not drive MISO.
  uint8_t idle = sdSpi.transfer(0xFF);
  Log.infoln("MISO idle byte with CS high: 0x%x %s", idle,
             idle == 0xFF ? "(ok, pulled/driven high)"
             : idle == 0x00 ? "(stuck LOW - check MISO wiring / short)"
                            : "(noisy - floating MISO? add pull-up)");

  // CMD0: GO_IDLE_STATE
  uint8_t r1 = 0xFF;
  for (int attempt = 1; attempt <= 10; attempt++) {
    r1 = sendCmd(0, 0);
    csHigh();
    Log.infoln("CMD0  attempt %d -> R1=0x%x", attempt, r1);
    if (r1 == 0x01) break;
    delay(10);
  }
  if (r1 != 0x01) {
    Log.errorln("CMD0 failed, card did not enter idle state.");
    explainR1(r1);
    sdSpi.endTransaction();
    return false;
  }
  Log.noticeln("CMD0 OK: card is in SPI idle state");

  // CMD8: SEND_IF_COND (distinguishes SD v2+ from v1/MMC)
  bool v2 = false;
  uint8_t r7[4] = {0};
  r1 = sendCmd(8, 0x1AA);
  for (auto &b : r7) b = sdSpi.transfer(0xFF);
  csHigh();
  Log.infoln("CMD8  -> R1=0x%x R7=%x %x %x %x", r1, r7[0], r7[1], r7[2], r7[3]);
  if (r1 == 0x01) {
    if ((r7[2] & 0x0F) != 0x01 || r7[3] != 0xAA) {
      Log.errorln("CMD8 echo mismatch (voltage/check pattern). Signal integrity problem?");
      sdSpi.endTransaction();
      return false;
    }
    v2 = true;
    Log.noticeln("CMD8 OK: SD v2+ card, 2.7-3.6V accepted");
  } else if (r1 & 0x04) {
    Log.noticeln("CMD8 illegal: SD v1 or MMC card");
  } else {
    Log.errorln("CMD8 unexpected response");
    explainR1(r1);
  }

  // CMD58: READ_OCR (before init)
  uint8_t ocr[4];
  r1 = sendCmd(58, 0);
  for (auto &b : ocr) b = sdSpi.transfer(0xFF);
  csHigh();
  Log.infoln("CMD58 -> R1=0x%x OCR=%x %x %x %x", r1, ocr[0], ocr[1], ocr[2], ocr[3]);

  // ACMD41: SD_SEND_OP_COND, loop until card leaves idle
  uint32_t start = millis();
  int tries = 0;
  do {
    r1 = sendCmd(ACMD(41), v2 ? 0x40000000 : 0);
    csHigh();
    tries++;
    if (r1 == 0x00) break;
    delay(5);
  } while (millis() - start < 2000);
  Log.infoln("ACMD41 -> R1=0x%x after %d tries / %l ms", r1, tries, millis() - start);
  if (r1 != 0x00) {
    Log.errorln("ACMD41 timed out: card never finished init (power supply? card dead?)");
    explainR1(r1);
    sdSpi.endTransaction();
    return false;
  }
  Log.noticeln("ACMD41 OK: card initialized");

  // CMD58 again: CCS bit tells SDHC/SDXC (block addressed) vs SDSC (byte addressed)
  bool blockAddressed = false;
  r1 = sendCmd(58, 0);
  for (auto &b : ocr) b = sdSpi.transfer(0xFF);
  csHigh();
  blockAddressed = v2 && (ocr[0] & 0x40);
  Log.infoln("CMD58 -> R1=0x%x OCR=%x %x %x %x -> %s", r1, ocr[0], ocr[1], ocr[2], ocr[3],
             blockAddressed ? "SDHC/SDXC" : "SDSC");

  if (!blockAddressed) {
    // Force 512-byte blocks for SDSC
    r1 = sendCmd(16, 512);
    csHigh();
    Log.infoln("CMD16(512) -> R1=0x%x", r1);
  }

  // CMD10: CID
  uint8_t cid[16];
  r1 = sendCmd(10, 0);
  if (r1 == 0 && readDataBlock(cid, sizeof(cid))) {
    csHigh();
    char pnm[6] = {(char)cid[3], (char)cid[4], (char)cid[5], (char)cid[6], (char)cid[7], 0};
    uint32_t psn = ((uint32_t)cid[9] << 24) | (cid[10] << 16) | (cid[11] << 8) | cid[12];
    cardSerial = psn;
    uint16_t year = 2000 + (((cid[13] & 0x0F) << 4) | (cid[14] >> 4));
    Serial.printf("CID: MID=0x%02X OID=%c%c PNM=%s PRV=%d.%d PSN=0x%08lX MDT=%u/%u\r\n", cid[0],
                  cid[1], cid[2], pnm, cid[8] >> 4, cid[8] & 0x0F, (unsigned long)psn,
                  cid[14] & 0x0F, year);
  } else {
    csHigh();
    Log.warningln("CMD10 (CID) failed, R1=0x%x", r1);
  }

  // CMD9: CSD -> capacity
  uint8_t csd[16];
  r1 = sendCmd(9, 0);
  if (r1 == 0 && readDataBlock(csd, sizeof(csd))) {
    csHigh();
    uint64_t capacity = 0;
    if ((csd[0] >> 6) == 1) {  // CSD v2
      uint32_t cSize = ((uint32_t)(csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];
      capacity = (uint64_t)(cSize + 1) * 512ULL * 1024ULL;
    } else {  // CSD v1
      uint32_t readBlLen = csd[5] & 0x0F;
      uint32_t cSize = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | (csd[8] >> 6);
      uint32_t mult = ((csd[9] & 0x03) << 1) | (csd[10] >> 7);
      capacity = (uint64_t)(cSize + 1) << (mult + 2 + readBlLen);
    }
    Serial.printf("CSD v%d, capacity: %lu MiB\r\n", (csd[0] >> 6) + 1,
                  (unsigned long)(capacity >> 20));
  } else {
    csHigh();
    Log.warningln("CMD9 (CSD) failed, R1=0x%x", r1);
  }

  // CMD17: read sector 0 (MBR or boot sector)
  static uint8_t sector[512];
  r1 = sendCmd(17, 0);
  bool ok = (r1 == 0) && readDataBlock(sector, sizeof(sector));
  csHigh();
  sdSpi.endTransaction();
  if (!ok) {
    Log.errorln("CMD17 read of sector 0 failed, R1=0x%x", r1);
    return false;
  }
  Log.noticeln("CMD17 OK: read sector 0");
  hexDump(sector + 0x1B0, 0x50);  // partition table + signature
  if (sector[510] == 0x55 && sector[511] == 0xAA) {
    Log.noticeln("Boot signature 0x55AA present");
    if (sector[0] == 0xEB || sector[0] == 0xE9) {
      Log.infoln("Sector 0 looks like a FAT/exFAT boot sector (no MBR)");
    } else {
      Serial.printf("Partition 1 type: 0x%02X\r\n", sector[0x1BE + 4]);
    }
  } else {
    Log.warningln("No 0x55AA signature in sector 0 - card unformatted or read data corrupt");
  }
  return true;
}

// ---------------------------------------------------------------------------
// Stage 2: SdFat filesystem test
// ---------------------------------------------------------------------------
static void fillPattern(uint8_t *buf, size_t len, uint32_t seed) {
  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)((seed * 131 + i * 7) ^ (i >> 8));
}

struct XferResult {
  uint32_t writeUs = 0;
  uint32_t readUs = 0;
  float writeKiBs = 0;
  float readKiBs = 0;
  char fail[80] = "";
};

// Writes `blocks` x TEST_BLOCK_SIZE of a known pattern to `path`, reads it back and
// compares. Returns false (with r.fail set) on any write/sync/read/compare error.
static bool writeReadVerify(const char *path, size_t blocks, XferResult &r) {
  static uint8_t buf[TEST_BLOCK_SIZE];
  static uint8_t expect[TEST_BLOCK_SIZE];
  FsFile f;

  if (!f.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
    snprintf(r.fail, sizeof(r.fail), "open %s for write failed (sd err 0x%02X)", path,
             sd.sdErrorCode());
    return false;
  }
  uint32_t t0 = micros();
  for (size_t i = 0; i < blocks; i++) {
    fillPattern(buf, sizeof(buf), i);
    if (f.write(buf, sizeof(buf)) != sizeof(buf)) {
      snprintf(r.fail, sizeof(r.fail), "write failed at block %u (sd err 0x%02X)", (unsigned)i,
               sd.sdErrorCode());
      f.close();
      return false;
    }
  }
  if (!f.sync()) {
    snprintf(r.fail, sizeof(r.fail), "sync failed (sd err 0x%02X)", sd.sdErrorCode());
    f.close();
    return false;
  }
  r.writeUs = micros() - t0;
  f.close();

  if (!f.open(path, O_RDONLY)) {
    snprintf(r.fail, sizeof(r.fail), "reopen %s for read failed (sd err 0x%02X)", path,
             sd.sdErrorCode());
    return false;
  }
  size_t mismatches = 0;
  for (size_t i = 0; i < blocks; i++) {
    uint32_t r0 = micros();
    int n = f.read(buf, sizeof(buf));
    r.readUs += micros() - r0;
    if (n != (int)sizeof(buf)) {
      snprintf(r.fail, sizeof(r.fail), "short read at block %u: %d bytes (sd err 0x%02X)",
               (unsigned)i, n, sd.sdErrorCode());
      f.close();
      return false;
    }
    fillPattern(expect, sizeof(expect), i);
    if (memcmp(buf, expect, sizeof(buf)) != 0) mismatches++;
  }
  f.close();

  float kib = (blocks * TEST_BLOCK_SIZE) / 1024.0f;
  r.writeKiBs = kib / (r.writeUs / 1e6f);
  r.readKiBs = kib / (r.readUs / 1e6f);
  if (mismatches) {
    snprintf(r.fail, sizeof(r.fail), "verify failed: %u of %u blocks corrupt",
             (unsigned)mismatches, (unsigned)blocks);
    return false;
  }
  return true;
}

// SPI1 kernel clock (120 MHz on the Nucleo-H753ZI variant). Real SCK = this / 2^n.
static uint32_t spiKernelHz() { return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123); }

// SCK actually programmed into SPI1 right now (read back from the MBR prescaler bits).
static uint32_t spiActualHz() {
  uint32_t mbr = (SPI1->CFG1 & SPI_CFG1_MBR) >> SPI_CFG1_MBR_Pos;
  return spiKernelHz() / (2UL << mbr);
}

static bool mountAt(uint32_t hz) {
  sd.end();
  return sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, hz, &sdSpi));
}

static bool fileSystemTest() {
  Log.infoln("===== Stage 2: SdFat filesystem @ %d MHz =====", FAST_CLOCK_MHZ);
  sdSpi.end();

  if (!sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(FAST_CLOCK_MHZ), &sdSpi))) {
    Log.errorln("sd.begin failed: errorCode=0x%x errorData=0x%x", sd.sdErrorCode(),
                sd.sdErrorData());
    Serial.print("  ");
    sd.printSdError(&Serial);
    Log.errorln("If stage 1 passed, try a lower FAST_CLOCK_MHZ or reformat as FAT32/exFAT.");
    return false;
  }
  sdMounted = true;
  Log.noticeln("SPI1 kernel clock %l Hz, requested %d MHz -> actual SCK %l Hz", spiKernelHz(),
               FAST_CLOCK_MHZ, spiActualHz());

  const char *fsName = sd.fatType() == FAT_TYPE_EXFAT ? "exFAT" : "FAT";
  Log.noticeln("Mounted %s (FAT%d), cluster size %l bytes, %l MiB free", fsName, sd.fatType(),
               (uint32_t)(sd.bytesPerCluster()),
               (uint32_t)((uint64_t)sd.freeClusterCount() * sd.bytesPerCluster() >> 20));

  Log.infoln("Root directory:");
  sd.ls(&Serial, LS_SIZE);

  XferResult r;
  bool ok = writeReadVerify(TEST_FILE, TEST_BLOCK_COUNT, r);
  if (r.writeUs && r.readUs) {
    float totalKiB = (TEST_BLOCK_SIZE * TEST_BLOCK_COUNT) / 1024.0f;
    Serial.printf("Write: %.1f KiB in %lu ms -> %.1f KiB/s\r\n", totalKiB,
                  (unsigned long)(r.writeUs / 1000), r.writeKiBs);
    Serial.printf("Read:  %.1f KiB in %lu ms -> %.1f KiB/s\r\n", totalKiB,
                  (unsigned long)(r.readUs / 1000), r.readKiBs);
  }
  if (!ok) {
    Log.errorln("%s", r.fail);
    return false;
  }
  Log.noticeln("Verify OK: all %d blocks match", (int)TEST_BLOCK_COUNT);
  return true;
}

// ---------------------------------------------------------------------------
// Stage 3: readable text file for cross-device check
// ---------------------------------------------------------------------------
static bool readableFileTest(bool binaryVerified) {
  Log.infoln("===== Stage 3: write %s for checking on another device =====", READABLE_FILE);

  // Count previous entries so each run gets a new number.
  runNumber = 1;
  FsFile f;
  if (f.open(READABLE_FILE, O_RDONLY)) {
    char line[96];
    int n;
    while ((n = f.fgets(line, sizeof(line))) > 0) {
      if (strncmp(line, "Run #", 5) == 0) runNumber++;
    }
    f.close();
  }

  if (!f.open(READABLE_FILE, O_WRONLY | O_CREAT | O_APPEND)) {
    Log.errorln("Failed to open %s for appending", READABLE_FILE);
    return false;
  }
  f.printf("Run #%d\r\n", runNumber);
  f.printf("  Written by: Nucleo-H753ZI over SPI1 (CS=PB6 SCK=PA5 MISO=PA6 MOSI=PB5) @ %lu MHz\r\n",
           (unsigned long)FAST_CLOCK_MHZ);
  f.printf("  Uptime at write: %lu ms\r\n", (unsigned long)millis());
  f.printf("  Card serial (PSN): 0x%08lX\r\n", (unsigned long)cardSerial);
  f.printf("  Binary test: %s, %lu bytes, verify %s\r\n", TEST_FILE,
           (unsigned long)(TEST_BLOCK_SIZE * TEST_BLOCK_COUNT), binaryVerified ? "PASS" : "FAIL");
  f.printf("  Check line: %s\r\n\r\n", CHECK_LINE);
  bool ok = f.sync() && !f.getWriteError();
  f.close();
  if (!ok) {
    Log.errorln("Write/sync of %s failed", READABLE_FILE);
    return false;
  }

  // Read the whole file back and echo it, so you can compare with what the PC shows.
  if (!f.open(READABLE_FILE, O_RDONLY)) {
    Log.errorln("Failed to reopen %s", READABLE_FILE);
    return false;
  }
  Log.noticeln("Wrote run #%d. Current contents of %s (%l bytes):", runNumber, READABLE_FILE,
               (uint32_t)f.fileSize());
  Serial.println("----------------------------------------");
  int c;
  while ((c = f.read()) >= 0) Serial.write((uint8_t)c);
  Serial.println("----------------------------------------");
  f.close();

  Log.infoln("Now power off / remove the card and open %s on a PC.", READABLE_FILE);
  Log.infoln("You should see run #%d with the same text as above.", runNumber);
  return true;
}

// ---------------------------------------------------------------------------
// Stage 4: SPI clock speed sweep
// ---------------------------------------------------------------------------
struct SweepStep {
  uint32_t hz;
  uint32_t div;
  bool ok;
  float writeKiBs;
  float readKiBs;
  char fail[96];
};
static SweepStep sweepSteps[8];
static int sweepCount = 0;

static void printMHz(Print &p, uint32_t hz) {
  p.printf("%lu.%03lu MHz", (unsigned long)(hz / 1000000), (unsigned long)((hz / 1000) % 1000));
}

static void printSweepTable(Print &p, const char *eol) {
  p.printf("  %-12s %-6s %-12s %-12s %s%s", "SCK", "div", "write KiB/s", "read KiB/s", "result",
           eol);
  for (int i = 0; i < sweepCount; i++) {
    const SweepStep &st = sweepSteps[i];
    p.print("  ");
    printMHz(p, st.hz);
    p.printf("%*s/%-5lu ", st.hz >= 10000000 ? 1 : 2, "", (unsigned long)st.div);
    if (st.ok)
      p.printf("%-12.1f %-12.1f PASS%s", st.writeKiBs, st.readKiBs, eol);
    else
      p.printf("%-12s %-12s FAIL: %s%s", "-", "-", st.fail, eol);
  }
}

// Returns the highest SCK (Hz) that passed every pass, or 0 if none did.
static uint32_t speedSweep() {
  uint32_t kernel = spiKernelHz();
  Log.infoln("===== Stage 4: SPI speed sweep (kernel clock %l Hz) =====", kernel);
  Log.infoln("%d passes x %l KiB per speed, stopping at the first failure", SWEEP_PASSES,
             (uint32_t)(SWEEP_BLOCK_COUNT * TEST_BLOCK_SIZE / 1024));

  uint32_t bestHz = 0;
  sweepCount = 0;
  for (uint32_t div = 256; div >= 2 && sweepCount < 8; div /= 2) {
    uint32_t hz = kernel / div;
    if (hz < SWEEP_MIN_HZ) continue;

    SweepStep &st = sweepSteps[sweepCount++];
    st = SweepStep{hz, div, false, 0, 0, ""};
    Serial.print("--- ");
    printMHz(Serial, hz);
    Serial.printf(" (kernel/%lu)%s ---\r\n", (unsigned long)div,
                  hz > 25000000 ? "  [above SD spec 25 MHz default-speed limit]" : "");

    if (!mountAt(hz)) {
      snprintf(st.fail, sizeof(st.fail), "mount failed (sd err 0x%02X data 0x%02X)",
               sd.sdErrorCode(), (unsigned)sd.sdErrorData());
    } else {
      st.ok = true;
      float wSum = 0, rSum = 0;
      for (int pass = 1; pass <= SWEEP_PASSES; pass++) {
        XferResult r;
        bool ok = writeReadVerify(SWEEP_FILE, SWEEP_BLOCK_COUNT, r);
        if (pass == 1 && spiActualHz() != hz) {
          Log.warningln("  SPI1 actually running at %l Hz", spiActualHz());
        }
        if (!ok) {
          st.ok = false;
          snprintf(st.fail, sizeof(st.fail), "pass %d: %s", pass, r.fail);
          break;
        }
        Serial.printf("  pass %d: write %.1f KiB/s, read %.1f KiB/s, verify OK\r\n", pass,
                      r.writeKiBs, r.readKiBs);
        wSum += r.writeKiBs;
        rSum += r.readKiBs;
      }
      if (st.ok) {
        st.writeKiBs = wSum / SWEEP_PASSES;
        st.readKiBs = rSum / SWEEP_PASSES;
      }
    }

    if (!st.ok) {
      Log.errorln("  FAIL: %s", st.fail);
      break;
    }
    Log.noticeln("  PASS");
    bestHz = hz;
  }

  Log.infoln("Sweep results:");
  printSweepTable(Serial, "\r\n");
  if (bestHz == 0) {
    Log.errorln("No speed passed.");
  } else {
    Serial.print("Highest reliable SCK: ");
    printMHz(Serial, bestHz);
    Serial.println(sweepSteps[sweepCount - 1].ok ? "  (hit max prescaler, never failed)"
                                                 : "");
  }
  return bestHz;
}

// Remount at the best speed, clean up, and append the sweep table to READABLE_FILE.
static bool recordSweep(uint32_t bestHz) {
  uint32_t hz = bestHz ? bestHz : 400000;
  // A failed step can leave the card confused; retry the mount a couple of times.
  bool mounted = false;
  for (int i = 0; i < 3 && !mounted; i++) mounted = mountAt(hz);
  if (!mounted) {
    Log.errorln("Could not remount after sweep - power-cycle the card to recover.");
    return false;
  }
  sd.remove(SWEEP_FILE);

  FsFile f;
  if (!f.open(READABLE_FILE, O_WRONLY | O_CREAT | O_APPEND)) {
    Log.errorln("Failed to open %s to record sweep", READABLE_FILE);
    return false;
  }
  f.printf("Run #%d speed sweep (%d x %lu KiB per speed):\r\n", runNumber, SWEEP_PASSES,
           (unsigned long)(SWEEP_BLOCK_COUNT * TEST_BLOCK_SIZE / 1024));
  printSweepTable(f, "\r\n");
  f.print("  Highest reliable SCK: ");
  if (bestHz)
    printMHz(f, bestHz);
  else
    f.print("none");
  f.print("\r\n\r\n");
  bool ok = f.sync() && !f.getWriteError();
  f.close();
  if (ok) Log.noticeln("Sweep results appended to %s", READABLE_FILE);
  return ok;
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  delay(500);

  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix([](Print *p, int level) { p->printf("[ %lu ] ", millis()); });

  pinMode(LED_GREEN, OUTPUT);
  Log.infoln("SD card SPI1 test - Nucleo-H753ZI");
  Log.infoln("CS=PB6 SCK=PA5 MISO=PA6 MOSI=PB5");

  bool raw = rawProtocolTest();
  bool fs = raw && fileSystemTest();
  bool txt = sdMounted && readableFileTest(fs);
  uint32_t bestHz = 0;
  bool swept = false;
  if (sdMounted) {
    bestHz = speedSweep();
    swept = true;
    recordSweep(bestHz);
  }

  Log.infoln("===== Summary =====");
  Log.infoln("Raw protocol: %s", raw ? "PASS" : "FAIL");
  Log.infoln("Filesystem:   %s", raw ? (fs ? "PASS" : "FAIL") : "SKIPPED");
  Log.infoln("Readable file: %s", sdMounted ? (txt ? "PASS" : "FAIL") : "SKIPPED");
  if (!swept)
    Log.infoln("Speed sweep:  SKIPPED");
  else if (bestHz)
    Log.infoln("Speed sweep:  max reliable SCK %l Hz", bestHz);
  else
    Log.infoln("Speed sweep:  no speed passed");
  Log.infoln("Press the black RESET button to rerun.");
}

void loop() {
  // Blink LD1 (green) slowly: test finished.
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
  delay(500);
}
