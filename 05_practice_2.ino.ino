int ledPin = 7;  // LED가 연결된 핀 번호

void setup() {
  pinMode(ledPin, OUTPUT);  // 7번 핀을 출력 모드로 설정
}

void loop() {
  // 처음 1초 동안 LED를 켜기
  digitalWrite(ledPin, LOW);  
  delay(1000);  // 1초 대기


  for (int i = 0; i < 6; i++) {
    digitalWrite(ledPin, LOW);  // LED 끄기
    delay(200);  // 0.2초 대기
    digitalWrite(ledPin, HIGH);  // LED 켜기
    delay(200);  // 0.2초 대기
  }

  digitalWrite(ledPin, HIGH);  // LED 끄기

  while(1) {
  }
}
