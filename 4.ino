#define TX_HIGH() (PORTB |=  (1 << PB0))
#define TX_LOW()  (PORTB &= ~(1 << PB0))
#define BIT_DELAY_US 104   // 9600 bps

void setup() {
  // D8(PB0) 출력 설정
  DDRB |= (1 << PB0);
  TX_HIGH();   // UART 유휴 상태

  // USB 시리얼 (D0, D1)
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
  // 1️⃣ D8에서 문자 전송
  sendByte('a');

  // 2️⃣ D0에서 수신되었는지 확인
  if (Serial.available()) {
    char rx = Serial.read();

    // 3️⃣ 받은 데이터를 PC 터미널로 출력
    Serial.write(rx);
  }

  delay(1000);  // 1초 간격 (관찰용)
}

/*
사용 핀 정리
역할	Arduino 핀	설명
송신(TX)	D8	비트뱅 UART 출력
수신(RX)	D0	하드웨어 UART RX
USB TX	D1	PC 터미널 출력
GND	GND	공통 접지
*/