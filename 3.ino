#define TX_HIGH() (PORTB |=  (1 << PB0))
#define TX_LOW()  (PORTB &= ~(1 << PB0))
#define BIT_DELAY_US 104

void setup() {
  // D8(PB0) 출력
  DDRB |= (1 << PB0);
  TX_HIGH();

  // 하드웨어 UART 수신용
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
  sendByte('a');   // ASCII 0x61
  delay(1000);     // 관찰용
}


/*
🔌 사용 핀 정리
역할	Arduino 핀	MCU 기준
송신(TX)	D8	PB0 (PORTB)
수신(RX)	D0	PD0
공통 접지	GND	GND
*/