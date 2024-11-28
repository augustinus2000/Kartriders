#include <SoftwareSerial.h>

SoftwareSerial BTSerial(12, 13); // 소프트웨어 시리얼 사용 (TX=12, RX=13)

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
  Serial.begin(9600); // 시리얼 모니터와 통신
  BTSerial.begin(9600);    // 블루투스 모듈과 통신
  Serial.println("블루투스 준비!");
}

void loop() {
  if (BTSerial.available()) {
    char command = BTSerial.read();  // 블루투스 데이터 수신
    Serial.print("받은 명령: ");  // 디버깅용 출력
    Serial.println(command);  

    switch (command) {
      case 'f': // 전진
        Serial.println("전진"); // println은 시리얼 모니터에 출력
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
        Serial.println("알 수 없는 명령");
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
