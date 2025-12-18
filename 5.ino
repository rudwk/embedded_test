#define TX_HIGH() (PORTB |=  (1 << PB0))
#define TX_LOW()  (PORTB &= ~(1 << PB0))
#define BIT_DELAY_US 104   // 9600 bps

char testChars[] = { 'a', 'A', 'Z' };
int indexChar = 0;

void setup() {
  // D8(PB0) 출력
  DDRB |= (1 << PB0);
  TX_HIGH();

  // USB 시리얼
  Serial.begin(9600);
}

void sendByte(uint8_t data) {
  // Start bit
  TX_LOW();
  delayMicroseconds(BIT_DELAY_US);

  // Data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    if (data & (1 << i))
      TX_HIGH();
    else
      TX_LOW();

    delayMicroseconds(BIT_DELAY_US);
  }

  // Stop bit
  TX_HIGH();
  delayMicroseconds(BIT_DELAY_US);
}

void loop() {
  // 1️⃣ 문자 하나 선택
  char txChar = testChars[indexChar];

  // 2️⃣ D8에서 송신
  sendByte(txChar);

  // 3️⃣ D0에서 수신 → PC 출력
  if (Serial.available()) {
    char rxChar = Serial.read();
    Serial.write(rxChar);
  }

  // 4️⃣ 다음 문자로 이동
  indexChar++;
  if (indexChar >= 3) indexChar = 0;

  delay(1000);
}

/*
사용 핀 정리
역할	Arduino 핀	설명
송신(TX)	D8	비트뱅 UART 출력
수신(RX)	D0	하드웨어 UART RX
USB TX	D1	PC 터미널 출력
GND	GND	공통 접지
*/