/*
 * SP/06 - 정규 속도 단계별 상승 실험 (v2: RX 실제값 로그 출력 추가)
 * Board: Arduino Uno
 * Connection: D8 ──── D0 (Loopback)
 *
 * 표준 보드레이트를 순차적으로 테스트
 * 각 속도에서 성공률과 비트폭(이론) 비교
 *
 * + 추가(v2):
 *   - 각 테스트에서 TX(기대) / RX(실제 수신) 값을 문자 + HEX로 출력
 *   - NO DATA(타임아웃)도 구분해서 표시
 *
 * + 추가(v2.1):
 *   - RX 문자를 '.'로 가공하지 않고, 실제 받은 바이트를 그대로 출력(Serial.write)
 *
 * 주의:
 * - 시리얼 모니터는 9600으로 고정 권장 (이 코드는 결과 출력 때만 9600으로 복귀)
 */

#define TX_BIT 0  // D8 = PORTB0

#define TX_HIGH() (PORTB |= (1 << TX_BIT))
#define TX_LOW()  (PORTB &= ~(1 << TX_BIT))

// ===============================
// (1) 테스트할 보드레이트 배열
// ===============================
// 표준 속도(기본)
long baudRates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
int numRates = 8;

// 만약 고속을 “형식상” 테스트하려면 아래로 교체 가능(대부분 실패/NO DATA 정상일 수 있음)
// long baudRates[] = {230400, 460800, 921600, 1000000, 2000000};
// int numRates = 5;

int currentRateIndex = 0;

// 테스트 문자
char testChars[] = {'a', 'A', '0', '9', '!'};
int numTestChars = 5;

// 각 보드레이트별 통계
struct BaudStats {
  long baudRate;
  unsigned long bitDelay;
  int totalTests;
  int successCount;
  int errorCount;
  int noDataCount;
  float successRate;
};

BaudStats stats[8]; // numRates 최대 8 기준(표준 배열). 고속 5개로 바꿔도 문제 없음.

static inline void printHex2(uint8_t v) {
  if (v < 16) Serial.print("0");
  Serial.print(v, HEX);
}

void setup() {
  // D8 출력 설정
  DDRB |= (1 << TX_BIT);
  TX_HIGH();

  // 초기 UART (출력용 9600)
  Serial.begin(9600);

  Serial.println("========================================");
  Serial.println(" SP/06 - Baud Rate Stepping Test (v2.1)");
  Serial.println("========================================");
  Serial.println("Connection: D8 ──── D0");
  Serial.println("PC Serial Monitor: 9600 bps (recommended)");
  Serial.println();

  Serial.println("Testing Baud Rates:");
  for (int i = 0; i < numRates; i++) {
    Serial.print("  ");
    Serial.print(baudRates[i]);
    Serial.print(" bps (bit width: ");
    Serial.print(1000000.0 / baudRates[i], 2);
    Serial.println(" us)");
  }
  Serial.println("========================================");
  Serial.println();
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  Serial.println("\nStarting tests...\n");
  delay(500);
}

void sendByteBitBang(char c, unsigned long bitDelay) {
  // Start bit
  TX_LOW();
  delayMicroseconds(bitDelay);

  // Data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    if ((c >> i) & 0x01) TX_HIGH();
    else                 TX_LOW();
... (263줄 남음)
접기
message.txt
11KB
/*
 * SP/06-HS - High-speed limit exploration (BitBang TX with Timer1 ticks + HW UART RX)
 * Board: Arduino Uno (ATmega328P 16MHz)
 * Connection: D8 ---- D0 (Loopback)
 *
 * Baud stepping:
확장
message.txt
10KB
﻿
지성
gwono8
 
/*
 * SP/06 - 정규 속도 단계별 상승 실험 (v2: RX 실제값 로그 출력 추가)
 * Board: Arduino Uno
 * Connection: D8 ──── D0 (Loopback)
 *
 * 표준 보드레이트를 순차적으로 테스트
 * 각 속도에서 성공률과 비트폭(이론) 비교
 *
 * + 추가(v2):
 *   - 각 테스트에서 TX(기대) / RX(실제 수신) 값을 문자 + HEX로 출력
 *   - NO DATA(타임아웃)도 구분해서 표시
 *
 * + 추가(v2.1):
 *   - RX 문자를 '.'로 가공하지 않고, 실제 받은 바이트를 그대로 출력(Serial.write)
 *
 * 주의:
 * - 시리얼 모니터는 9600으로 고정 권장 (이 코드는 결과 출력 때만 9600으로 복귀)
 */

#define TX_BIT 0  // D8 = PORTB0

#define TX_HIGH() (PORTB |= (1 << TX_BIT))
#define TX_LOW()  (PORTB &= ~(1 << TX_BIT))

// ===============================
// (1) 테스트할 보드레이트 배열
// ===============================
// 표준 속도(기본)
long baudRates[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
int numRates = 8;

// 만약 고속을 “형식상” 테스트하려면 아래로 교체 가능(대부분 실패/NO DATA 정상일 수 있음)
// long baudRates[] = {230400, 460800, 921600, 1000000, 2000000};
// int numRates = 5;

int currentRateIndex = 0;

// 테스트 문자
char testChars[] = {'a', 'A', '0', '9', '!'};
int numTestChars = 5;

// 각 보드레이트별 통계
struct BaudStats {
  long baudRate;
  unsigned long bitDelay;
  int totalTests;
  int successCount;
  int errorCount;
  int noDataCount;
  float successRate;
};

BaudStats stats[8]; // numRates 최대 8 기준(표준 배열). 고속 5개로 바꿔도 문제 없음.

static inline void printHex2(uint8_t v) {
  if (v < 16) Serial.print("0");
  Serial.print(v, HEX);
}

void setup() {
  // D8 출력 설정
  DDRB |= (1 << TX_BIT);
  TX_HIGH();

  // 초기 UART (출력용 9600)
  Serial.begin(9600);

  Serial.println("========================================");
  Serial.println(" SP/06 - Baud Rate Stepping Test (v2.1)");
  Serial.println("========================================");
  Serial.println("Connection: D8 ──── D0");
  Serial.println("PC Serial Monitor: 9600 bps (recommended)");
  Serial.println();

  Serial.println("Testing Baud Rates:");
  for (int i = 0; i < numRates; i++) {
    Serial.print("  ");
    Serial.print(baudRates[i]);
    Serial.print(" bps (bit width: ");
    Serial.print(1000000.0 / baudRates[i], 2);
    Serial.println(" us)");
  }
  Serial.println("========================================");
  Serial.println();
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  Serial.println("\nStarting tests...\n");
  delay(500);
}

void sendByteBitBang(char c, unsigned long bitDelay) {
  // Start bit
  TX_LOW();
  delayMicroseconds(bitDelay);

  // Data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    if ((c >> i) & 0x01) TX_HIGH();
    else                 TX_LOW();
    delayMicroseconds(bitDelay);
  }

  // Stop bit
  TX_HIGH();
  delayMicroseconds(bitDelay);
}

void testBaudRate(long baudRate, int testIndex) {
  // 5글자 x 3회 = 15회 로그 저장
  const int repeats = 3;
  const int maxTests = repeats * 5; // 15

  char sentLog[maxTests];
  uint8_t recvLog[maxTests];     // 실제 수신 바이트(원시)
  bool gotDataLog[maxTests];     // 수신 유무
  bool okLog[maxTests];          // 기대와 일치 여부
  int logIndex = 0;

  // (A) 테스트 안내/결과 출력은 항상 9600
  Serial.end();
  delay(100);
  Serial.begin(9600);
  delay(100);

  Serial.println("========================================");
  Serial.print("TEST #");
  Serial.print(testIndex + 1);
  Serial.print(": ");
  Serial.print(baudRate);
  Serial.println(" bps");
  Serial.println("========================================");

  unsigned long bitDelay = 1000000UL / (unsigned long)baudRate;  // 정수 버림(원본 유지)

  Serial.print("Bit Width (Theory): ");
  Serial.print(1000000.0 / baudRate, 2);
  Serial.println(" us");
  Serial.print("Bit Delay (int): ");
  Serial.print(bitDelay);
  Serial.println(" us");
  Serial.println();

  // (B) RX(UART)를 테스트 보레이트로 설정 (여기부터는 출력 최소화 권장)
  Serial.end();
  delay(100);
  Serial.begin(baudRate);
  delay(100);

  // RX 버퍼 비우기
  while (Serial.available() > 0) {
    Serial.read();
  }

  int testSuccess = 0;
  int testErrors = 0;
  int noDataCount = 0;

  // (C) 테스트 (5문자 x 3회)
  for (int repeat = 0; repeat < repeats; repeat++) {
    for (int i = 0; i < numTestChars; i++) {
      char testChar = testChars[i];

      // 로그 기본값 세팅
      if (logIndex < maxTests) {
        sentLog[logIndex] = testChar;
        recvLog[logIndex] = 0;
        gotDataLog[logIndex] = false;
        okLog[logIndex] = false;
      }

      // 비트뱅 송신 (D8)
      sendByteBitBang(testChar, bitDelay);

      // 수신 대기
      unsigned long timeout = millis() + 100;
      while (Serial.available() == 0 && millis() < timeout) {
        delayMicroseconds(10);
      }

      // 수신 확인
      if (Serial.available() > 0) {
        uint8_t received = (uint8_t)Serial.read();

        if (logIndex < maxTests) {
          recvLog[logIndex] = received;
          gotDataLog[logIndex] = true;
        }

        if ((char)received == testChar) {
          testSuccess++;
          if (logIndex < maxTests) okLog[logIndex] = true;
        } else {
          testErrors++;
          if (logIndex < maxTests) okLog[logIndex] = false;
        }
      } else {
        testErrors++;
        noDataCount++;
        // gotDataLog=false 유지
      }

      if (logIndex < maxTests) logIndex++;

      delay(20);
    }
  }

  // 통계 저장
  stats[testIndex].baudRate = baudRate;
  stats[testIndex].bitDelay = bitDelay;
  stats[testIndex].totalTests = testSuccess + testErrors;
  stats[testIndex].successCount = testSuccess;
  stats[testIndex].errorCount = testErrors;
  stats[testIndex].noDataCount = noDataCount;
  stats[testIndex].successRate =
      (stats[testIndex].totalTests > 0)
          ? (testSuccess * 100.0) / (float)stats[testIndex].totalTests
          : 0.0;

  // (D) 결과 출력 위해 9600 복귀
  Serial.end();
  delay(100);
  Serial.begin(9600);
  delay(100);

  // 결과 출력
  Serial.println("----------------------------------------");
  Serial.println("Results:");
  Serial.print("  Total Tests: ");
  Serial.println(stats[testIndex].totalTests);
  Serial.print("  Success: ");
  Serial.print(testSuccess);
  Serial.print(" (");
  Serial.print(stats[testIndex].successRate, 1);
  Serial.println("%)");
  Serial.print("  Errors: ");
  Serial.print(testErrors);
  Serial.print(" (");
  Serial.print((testErrors * 100.0) / (float)stats[testIndex].totalTests, 1);
  Serial.println("%)");
  Serial.print("  No Data: ");
  Serial.println(noDataCount);

  // ===== 실제 수신값 로그 출력(핵심) =====
  Serial.println();
  Serial.println("Per-character log (TX expected vs RX actual):");
  for (int k = 0; k < logIndex; k++) {
    Serial.print("  #");
    if (k + 1 < 10) Serial.print("0");
    Serial.print(k + 1);

    // TX
    Serial.print("  TX='");
    Serial.print(sentLog[k]);
    Serial.print("' (0x");
    printHex2((uint8_t)sentLog[k]);
    Serial.print(")  ");

    // RX (★ 여기만 변경: 받은 바이트 그대로 출력)
    Serial.print("RX=");
    if (!gotDataLog[k]) {
      Serial.print("<NO DATA>");
    } else {
      uint8_t r = recvLog[k];
      Serial.print("'");
      Serial.write(r);          // ★ 깨진 바이트 그대로 PC로 출력
      Serial.print("' (0x");
      printHex2(r);
      Serial.print(")");
    }

    Serial.print("  -> ");
    Serial.println(okLog[k] ? "OK" : "FAIL");
  }
  Serial.println("========================================");
  Serial.println();

  delay(2000);
}

void printFinalSummary() {
  Serial.println("\n\n");
  Serial.println("########################################");
  Serial.println("#         FINAL SUMMARY TABLE          #");
  Serial.println("########################################");
  Serial.println();
  Serial.println("Baud Rate | Bit Width | Tests | Success | Errors | NoData | Rate");
  Serial.println("----------|-----------|-------|---------|--------|--------|------");

  for (int i = 0; i < numRates; i++) {
    // Baud Rate
    if (baudRates[i] < 10000) Serial.print(" ");
    if (baudRates[i] < 100000) Serial.print(" ");
    Serial.print(baudRates[i]);
    Serial.print(" | ");

    // Bit Width
    float bitWidth = 1000000.0 / baudRates[i];
    if (bitWidth < 100) Serial.print(" ");
    if (bitWidth < 10) Serial.print(" ");
    Serial.print(bitWidth, 2);
    Serial.print(" us | ");

    // Tests
    if (stats[i].totalTests < 10) Serial.print(" ");
    Serial.print(stats[i].totalTests);
    Serial.print("    | ");

    // Success
    if (stats[i].successCount < 10) Serial.print(" ");
    Serial.print(stats[i].successCount);
    Serial.print("      | ");

    // Errors
    if (stats[i].errorCount < 10) Serial.print(" ");
    Serial.print(stats[i].errorCount);
    Serial.print("     | ");

    // NoData
    if (stats[i].noDataCount < 10) Serial.print(" ");
    Serial.print(stats[i].noDataCount);
    Serial.print("     | ");

    // Rate
    if (stats[i].successRate < 100) Serial.print(" ");
    Serial.print(stats[i].successRate, 1);
    Serial.println("%");
  }

  Serial.println("########################################");
  Serial.println();
  Serial.println("CSV Format (for data analysis):");
  Serial.println("baud_rate,bit_width_us,total_tests,success,errors,nodata,success_rate");

  for (int i = 0; i < numRates; i++) {
    Serial.print(stats[i].baudRate);
    Serial.print(",");
    Serial.print(1000000.0 / stats[i].baudRate, 2);
    Serial.print(",");
    Serial.print(stats[i].totalTests);
    Serial.print(",");
    Serial.print(stats[i].successCount);
    Serial.print(",");
    Serial.print(stats[i].errorCount);
    Serial.print(",");
    Serial.print(stats[i].noDataCount);
    Serial.print(",");
    Serial.println(stats[i].successRate, 2);
  }

  Serial.println("\nTest Complete!");
}

void loop() {
  if (currentRateIndex < numRates) {
    testBaudRate(baudRates[currentRateIndex], currentRateIndex);
    currentRateIndex++;
  } else {
    printFinalSummary();
    while (true) delay(1000);
  }
}
message.txt
11KB