#include <SoftwareSerial.h>

// RX는 사용 안 하므로 -1, TX를 D8로 설정
SoftwareSerial mySerial(-1, 8);

void setup() {
  mySerial.begin(9600);   // 9600 bps 설정
}

void loop() {
  mySerial.write('a');    // 문자 'a' 전송
  delay(1000);            // 1초마다 반복 (파형 관찰용)
}
/*
✔ Arduino Uno 핀
용도	핀
UART TX	D8 (SoftwareSerial 사용)
GND	GND

⚠ 주의

Arduino Uno의 기본 TX는 D1이지만,
관찰 실습에서는 SoftwareSerial을 써서 D8을 TX로 쓰는 게 좋음

그래야 USB 시리얼과 충돌 없음

✔ 오실로스코프

CH1 → D8

GND → Arduino GND
*/