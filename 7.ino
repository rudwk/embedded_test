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
 * SP/06-HS - High-speed limit exploration (BitBang TX with Timer1 ticks + HW UART RX)
 * Board: Arduino Uno (ATmega328P 16MHz)
 * Connection: D8 ---- D0 (Loopback)
 *
 * Baud stepping:
 * 230400 → 460800 → 921600 → 1000000(1Mbps) → 2000000(2Mbps)
 *
 * Output to PC: Serial Monitor 9600 bps (fixed)
 *
 * RX log:
 * - TX expected: char + HEX
 * - RX actual  : RAW byte 그대로 Serial.write + HEX
 * - NO DATA 구분
 */

#define TX_BIT 0  // D8 = PORTB0
#define TX_HIGH() (PORTB |=  (1 << TX_BIT))
#define TX_LOW()  (PORTB &= ~(1 << TX_BIT))

// ===============================
// (1) 고속 단계 (요구사항 그대로)
// ===============================
long baudRates[] = {230400, 460800, 921600, 1000000, 2000000};
const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);
int currentRateIndex = 0;

// 테스트 문자
char testChars[] = {'a', 'A', '0', '9', '!'};
const int numTestChars = sizeof(testChars) / sizeof(testChars[0]);
const int repeats = 3;                // 5글자 x 3회 = 15회
const int maxTests = numTestChars * repeats;

struct BaudStats {
  long baudRate;
  float bitWidthUs;
  uint16_t bitTicks;     // Timer1 ticks per bit (rounded)
  int totalTests;
  int successCount;
  int errorCount;
  int noDataCount;
  float successRate;
};

BaudStats stats[numRates];

static inline void printHex2(uint8_t v) {
  if (v < 16) Serial.print("0");
  Serial.print(v, HEX);
}

// ===============================
// Timer1 tick (prescaler=1)
// 16MHz => 1 tick = 62.5ns
// ===============================
static inline void timer1_init_prescaler_1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10); // prescaler=1
  TCNT1 = 0;
}

static inline void waitTicks(uint16_t ticks) {
  uint16_t start = TCNT1;
  while ((uint16_t)(TCNT1 - start) < ticks) { /* busy wait */ }
}

// rounded ticks = (F_CPU + baud/2) / baud
static inline uint16_t baudToBitTicks(long baud) {
  uint32_t t = ((uint32_t)F_CPU + (uint32_t)(baud / 2)) / (uint32_t)baud;
  if (t == 0) t = 1;
  if (t > 65535) t = 65535;
  return (uint16_t)t;
}

// ===============================
// BitBang TX on D8 (8N1)
// ===============================
void sendByteBitBang_Ticks(char c, uint16_t bitTicks) {
  // Start bit
  TX_LOW();
  waitTicks(bitTicks);

  // Data bits LSB first
  for (int i = 0; i < 8; i++) {
    if ((c >> i) & 0x01) TX_HIGH();
    else                 TX_LOW();
    waitTicks(bitTicks);
  }

  // Stop bit
  TX_HIGH();
  waitTicks(bitTicks);
}

void setup() {
  // D8 output
  DDRB |= (1 << TX_BIT);
  TX_HIGH();

  // Timer1 init
  timer1_init_prescaler_1();

  // PC output fixed 9600
  Serial.begin(9600);
  delay(200);

  Serial.println("========================================");
  Serial.println(" SP/06-HS - High-Speed Limit Exploration");
  Serial.println("========================================");
  Serial.println("TX: BitBang on D8 (Timer1 ticks)");
  Serial.println("RX: HW UART on D0");
  Serial.println("Loopback: D8 ---- D0");
  Serial.println("PC Serial Monitor: 9600 (fixed)");
  Serial.println();

  Serial.println("Testing Baud Rates:");
  for (int i = 0; i < numRates; i++) {
    Serial.print("  ");
    Serial.print(baudRates[i]);
    Serial.print(" bps (bit width: ");
    Serial.print(1000000.0 / (float)baudRates[i], 3);
    Serial.println(" us)");
  }

  Serial.println("========================================");
  Serial.println("Starting in 3 seconds...");
  delay(3000);
  Serial.println("\nStarting tests...\n");
}

void testBaudRate(long baudRate, int testIndex) {
  // per-character logs
  char sentLog[maxTests];
  uint8_t recvLog[maxTests];
  bool gotDataLog[maxTests];
  bool okLog[maxTests];
  int logIndex = 0;

  // 안내/결과 출력은 9600
  Serial.end();
  delay(80);
  Serial.begin(9600);
  delay(80);

  Serial.println("========================================");
  Serial.print("TEST #");
  Serial.print(testIndex + 1);
  Serial.print(": ");
  Serial.print(baudRate);
  Serial.println(" bps");
  Serial.println("========================================");

  float bitWidthUs = 1000000.0 / (float)baudRate;
  uint16_t bitTicks = baudToBitTicks(baudRate);

  Serial.print("Bit Width (Theory): ");
  Serial.print(bitWidthUs, 3);
  Serial.println(" us");

  Serial.print("Timer1 bitTicks (rounded): ");
  Serial.print(bitTicks);
  Serial.print(" ticks  (tick=");
  Serial.print(1000000.0 / (float)F_CPU, 5);
  Serial.println(" us)");
  Serial.println();

  // RX UART를 테스트 보레이트로 설정 (이 구간에서는 출력 금지)
  Serial.end();
  delay(120);
  Serial.begin(baudRate);
  delay(120);

  while (Serial.available() > 0) (void)Serial.read();

  int testSuccess = 0;
  int testErrors  = 0;
  int noDataCount = 0;

  // 5문자 x 3회
  for (int r = 0; r < repeats; r++) {
    for (int i = 0; i < numTestChars; i++) {
      char tx = testChars[i];

      // init log
      sentLog[logIndex] = tx;
      recvLog[logIndex] = 0;
      gotDataLog[logIndex] = false;
      okLog[logIndex] = false;

      // TX (bitbang ticks)
      sendByteBitBang_Ticks(tx, bitTicks);

      // RX wait (짧게)
      uint32_t start = micros();
      while (Serial.available() == 0) {
        if ((uint32_t)(micros() - start) > 5000UL) break; // 5ms
      }

      if (Serial.available() > 0) {
        uint8_t rx = (uint8_t)Serial.read();
        recvLog[logIndex] = rx;
        gotDataLog[logIndex] = true;

        if ((char)rx == tx) {
          testSuccess++;
          okLog[logIndex] = true;
        } else {
          testErrors++;
          okLog[logIndex] = false;
        }
      } else {
        testErrors++;
        noDataCount++;
      }

      logIndex++;

      // 프레임 간 간격(너무 짧으면 연속 영향 커짐)
      delayMicroseconds(200);
    }
  }

  // stats 저장
  stats[testIndex].baudRate = baudRate;
  stats[testIndex].bitWidthUs = bitWidthUs;
  stats[testIndex].bitTicks = bitTicks;
  stats[testIndex].totalTests = testSuccess + testErrors;
  stats[testIndex].successCount = testSuccess;
  stats[testIndex].errorCount = testErrors;
  stats[testIndex].noDataCount = noDataCount;
  stats[testIndex].successRate =
      (stats[testIndex].totalTests > 0)
        ? (testSuccess * 100.0f) / (float)stats[testIndex].totalTests
        : 0.0f;

  // 결과 출력 위해 9600 복귀
  Serial.end();
  delay(80);
  Serial.begin(9600);
  delay(80);

  Serial.println("----------------------------------------");
  Serial.println("Results:");
  Serial.print("  Total Tests: "); Serial.println(stats[testIndex].totalTests);
  Serial.print("  Success    : "); Serial.print(testSuccess);
  Serial.print(" ("); Serial.print(stats[testIndex].successRate, 1); Serial.println("%)");
  Serial.print("  Errors     : "); Serial.println(testErrors);
  Serial.print("  No Data    : "); Serial.println(noDataCount);

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

    // RX (깨진 바이트 그대로 출력)
    Serial.print("RX=");
    if (!gotDataLog[k]) {
      Serial.print("<NO DATA>");
    } else {
      uint8_t rcv = recvLog[k];
      Serial.print("'");
      Serial.write(rcv);           // ★ 그대로(깨진 문자 포함) 출력
      Serial.print("' (0x");
      printHex2(rcv);
      Serial.print(")");
    }

    Serial.print("  -> ");
    Serial.println(okLog[k] ? "OK" : "FAIL");
  }
  Serial.println("========================================");
  Serial.println();

  delay(1500);
}

void printFinalSummary() {
  Serial.println("\n########################################");
  Serial.println("#   FINAL SUMMARY (High-Speed Limit)   #");
  Serial.println("########################################");
  Serial.println("Baud     | Bit(us) | Ticks | Tests | OK | Err | NoData | Rate");
  Serial.println("---------|---------|-------|-------|----|-----|--------|------");

  long maxStable = 0;
  for (int i = 0; i < numRates; i++) {
    Serial.print(stats[i].baudRate); Serial.print(" | ");
    Serial.print(stats[i].bitWidthUs, 3); Serial.print(" | ");
    Serial.print(stats[i].bitTicks); Serial.print(" | ");
    Serial.print(stats[i].totalTests); Serial.print(" | ");
    Serial.print(stats[i].successCount); Serial.print(" | ");
    Serial.print(stats[i].errorCount); Serial.print(" | ");
    Serial.print(stats[i].noDataCount); Serial.print(" | ");
    Serial.print(stats[i].successRate, 1); Serial.println("%");

    if (stats[i].successRate == 100.0f) maxStable = stats[i].baudRate;
  }

  Serial.println("----------------------------------------");
  Serial.print("Maximum Stable Baud (100%): ");
  if (maxStable > 0) {
    Serial.print(maxStable);
    Serial.println(" bps");
  } else {
    Serial.println("< 230400 bps");
  }
  Serial.println("########################################");
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
10KB