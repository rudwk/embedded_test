#define TX_HIGH() (PORTB |=  (1 << PB0))
#define TX_LOW()  (PORTB &= ~(1 << PB0))
#define BIT_DELAY_US 104

void setup() {
  DDRB |= (1 << PB0);   // D8(PB0) 출력 설정
  TX_HIGH();            // UART 유휴 상태는 HIGH
}

void sendByte(uint8_t data) {
  // 스타트 비트 (LOW)
  TX_LOW();
  delayMicroseconds(BIT_DELAY_US);

  // 데이터 비트 8개 (LSB → MSB)
  for (int i = 0; i < 8; i++) {
    if (data & (1 << i))
      TX_HIGH();
    else
      TX_LOW();

    delayMicroseconds(BIT_DELAY_US);
  }

  // 스톱 비트 (HIGH)
  TX_HIGH();
  delayMicroseconds(BIT_DELAY_US);
}

void loop() {
  sendByte('a');   // 문자 'a' 전송
  delay(1000);     // 1초 대기 (파형 관찰용)
}
/*
실제 배선 연결 방법
✔ ① Arduino → 오실로스코프
오실로스코프	Arduino
CH1 프로브 팁	D8
CH1 GND 클립	GND

중요

GND를 연결하지 않으면 파형이 뜨거나 흔들리거나 이상하게 보임

반드시 같은 기준 접지(Common Ground) 필요
*/
