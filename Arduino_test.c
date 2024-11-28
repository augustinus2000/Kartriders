// BTS7960 핀
const int LPWM1 = 5;  // 왼쪽 PWM 입력 (전진)
const int RPWM1 = 6;  // 오른쪽 PWM 입력 (후진)
const int LEN1 = 2;   // 왼쪽 모터 활성화
const int REN1 = 4;   // 오른쪽 모터 활성화

const int LPWM2 = 9;  // 왼쪽 PWM 입력 (전진)
const int RPWM2 = 10; // 오른쪽 PWM 입력 (후진)
const int LEN2 = 7;   // 왼쪽 모터 활성화
const int REN2 = 8;   // 오른쪽 모터 활성화

void setup() {
  // 핀 설정
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);
  pinMode(LEN1, OUTPUT);
  pinMode(REN1, OUTPUT);

  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LEN2, OUTPUT);
  pinMode(REN2, OUTPUT);

  // BTS7960 모터 구동부 활성화 핀 초기화
  digitalWrite(LEN1, LOW);
  digitalWrite(REN1, LOW);
  digitalWrite(LEN2, LOW);
  digitalWrite(REN2, LOW);

  // 시리얼 통신 초기화
  Serial.begin(9600);
  Serial.println("BTS7960 모터 드라이버 초기화");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read(); // 시리얼 입력 받기

    switch (command) {
      case 'f': // 전진
        Serial.println("전진");
        moveForward(51); // 0~255가 PWM, 0이 최소, 255가 최대, 속도 51 -> 255의 20%로 설정
        break;
      case 'b': // 후진
        Serial.println("후진");
        moveBackward(51);
        break;
      case 'l': // 좌회전
        Serial.println("좌회전");
        turnLeft(25);
        break;
      case 'r': // 우회전
        Serial.println("우회전");
        turnRight(25);
        break;
      case 's': // 정지
        Serial.println("정지");
        stopMotors();
        break;
      default:
        Serial.println("아무 명령 X");
        stopMotors();
    }
  }
}

// 전진
void moveForward(int speed) {
  digitalWrite(LEN1, HIGH);  // 모터1 전진 활성화
  digitalWrite(REN1, LOW);   // 모터1 후진 비활성화
  digitalWrite(LEN2, HIGH);  // 모터2 전진 활성화
  digitalWrite(REN2, LOW);   // 모터2 후진 비활성화

  analogWrite(LPWM1, speed); // 모터1 전진 속도 설정
  analogWrite(RPWM1, LOW);   // 모터1 후진 속도 없음
  analogWrite(LPWM2, speed); // 모터2 전진 속도 설정
  analogWrite(RPWM2, LOW);   // 모터2 후진 속도 없음
  
}

// 후진
void moveBackward(int speed) {
  digitalWrite(LEN1, LOW);   // 모터1 전진 비활성화
  digitalWrite(REN1, HIGH);  // 모터1 후진 활성화
  digitalWrite(LEN2, LOW);   // 모터2 전진 비활성화
  digitalWrite(REN2, HIGH);  // 모터2 후진 활성화

  analogWrite(LPWM1, LOW);   // 모터1 전진 속도 없음
  analogWrite(RPWM1, speed); // 모터1 후진 속도 설정
  analogWrite(LPWM2, LOW);   // 모터2 전진 속도 없음
  analogWrite(RPWM2, speed); // 모터2 후진 속도 설정
}

// 좌회전
void turnLeft(int speed) {
  digitalWrite(LEN1, LOW);   // 모터1 전진 비활성화
  digitalWrite(REN1, HIGH);  // 모터1 후진 활성화
  digitalWrite(LEN2, HIGH);  // 모터2 전진 활성화
  digitalWrite(REN2, LOW);   // 모터2 후진 비활성화

  analogWrite(LPWM1, LOW);   // 모터1 전진 속도 없음
  analogWrite(RPWM1, speed); // 모터1 후진 속도 설정
  analogWrite(LPWM2, speed); // 모터2 전진 속도 설정
  analogWrite(RPWM2, LOW);   // 모터2 후진 속도 없음
}

// 우회전
void turnRight(int speed) {
  digitalWrite(LEN1, HIGH);  // 모터1 전진 활성화
  digitalWrite(REN1, LOW);   // 모터1 후진 비활성화
  digitalWrite(LEN2, LOW);   // 모터2 전진 비활성화
  digitalWrite(REN2, HIGH);  // 모터2 후진 활성화

  analogWrite(LPWM1, speed); // 모터1 전진 속도 설정
  analogWrite(RPWM1, LOW);   // 모터1 후진 속도 없음
  analogWrite(LPWM2, LOW);   // 모터2 전진 속도 없음
  analogWrite(RPWM2, speed); // 모터2 후진 속도 설정
}

// 정지
void stopMotors() {
  digitalWrite(LEN1, LOW);   // 모터1 전진 비활성화
  digitalWrite(REN1, LOW);   // 모터1 후진 비활성화
  digitalWrite(LEN2, LOW);   // 모터2 전진 비활성화
  digitalWrite(REN2, LOW);   // 모터2 후진 비활성화

  analogWrite(LPWM1, LOW);   // 모터1 전진 속도 없음
  analogWrite(RPWM1, LOW);   // 모터1 후진 속도 없음
  analogWrite(LPWM2, LOW);   // 모터2 전진 속도 없음
  analogWrite(RPWM2, LOW);   // 모터2 후진 속도 없음
}
