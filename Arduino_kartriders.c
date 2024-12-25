#include <SoftwareSerial.h>

SoftwareSerial HC_06(2, 3); // 소프트웨어 시리얼 사용
// 아두이노 RX = 2번, 아두이노 TX = 3번 -->> 블루투스 모듈 TX를 아두이노 2번에, 
//                                            블루투스 모듈 RX를 아두이노 3번에 넣기

const int LPWM1 = 5;  // 왼쪽 PWM 입력 (전진)
const int RPWM1 = 6;  // 오른쪽 PWM 입력 (후진)

const int LPWM2 = 9;  // 왼쪽 PWM 입력 (전진)
const int RPWM2 = 10; // 오른쪽 PWM 입력 (후진)

void setup() {
  // 핀 설정
  pinMode(LPWM1, OUTPUT);
  pinMode(RPWM1, OUTPUT);

  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);

  // 시리얼 통신 초기화
  // Serial.begin(9600); // 시리얼 모니터와 통신, 아두이노 IDE에서 볼 수 있는 UI창.
                      // 노트북과 아두이노를 usb로 연결해놔야 볼 수 있는데,
                      // 고장 원인이 파악이 안되어서 usb 연결은 안함. 없어도 되는 코드

  HC_06.begin(115200);    // 블루투스 모듈과 통신
}

void loop() {
  if (HC_06.available()) {
    String command = HC_06.readStringUntil('\n');  // '\n'으로 끝나는 문자열 읽기
    
    // 첫 번째 쉼표 위치 찾기
    int firstCommaIndex = command.indexOf(',');
    // 두 번째 쉼표 위치 찾기
    int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1);
    // python 코드에서 쉼표로 구분 했기 때문에 쉼표 위치를 찾음

    // 쉼표가 두 개 이상 있는지 확인
    if (firstCommaIndex != -1 && secondCommaIndex != -1) { // 쉼표 위치가 찾아졌으면
      String charSignal = command.substring(0, firstCommaIndex);                    // 첫 번째 쉼표 앞 문자
      String numSignal1 = command.substring(firstCommaIndex + 1, secondCommaIndex); // 첫 번째 쉼표와 두 번째 쉼표 사이 숫자
      String numSignal2 = command.substring(secondCommaIndex + 1);                  // 두 번째 쉼표 뒤 숫자
      
      char receivedChar = charSignal.charAt(0); // 문자로 변환
      int receivedNum1 = numSignal1.toInt();    // 첫 번째 숫자 변환
      int receivedNum2 = numSignal2.toInt();    // 두 번째 숫자 변환

      // 문자에 따라 모터 제어
      if (receivedChar == 'f') {  // 전진
        // Serial.println("전진");   // 마찬가지로 시리얼 모니터를 쓴다면 이 문장이 출력돼요.
        moveForward(receivedNum1);  // speed는 받은 숫자로 설정 -> pwm 값을 받았다!
      }
      else if (receivedChar == 'b') {  // 후진
        // Serial.println("후진");
        moveBackward(receivedNum1);
      }
      else if (receivedChar == 'l') {  // 좌회전
        // Serial.println("좌회전");
        turnLeft(receivedNum1);
      }
      else if (receivedChar == 'r') {  // 우회전
        // Serial.println("우회전");
        turnRight(receivedNum1);
      }
      else if (receivedChar == 's') {  // 정지
        // Serial.println("정지");
        stopMotors();
      }
      else if (receivedChar == 'z') {  // 전진 + 좌회전
        // Serial.println("전진 + 좌회전");
        forwardLeft(receivedNum1, receivedNum2);
      }
      else if (receivedChar == 'x') {  // 전진 + 우회전
        // Serial.println("전진 + 우회전");
        forwardRight(receivedNum1, receivedNum2);
      }
      else if (receivedChar == 'c') {  // 후진 + 좌회전
        // Serial.println("후진 + 좌회전");
        backwardLeft(receivedNum1, receivedNum2);
      }
      else if (receivedChar == 'v') {  // 후진 + 우회전
        // Serial.println("후진 + 우회전");
        backwardRight(receivedNum1, receivedNum2);
      }
      else {
        // Serial.println("알 수 없는 명령");
        stopMotors();
      }
    }
  }
}



// 전진
void moveForward(int speed) {
  analogWrite(LPWM1, speed);    // 모터1 전진 속도 설정
  analogWrite(RPWM1, LOW);      // 모터1 후진 속도 없음
  analogWrite(LPWM2, speed+27); // 모터2 전진 속도 설정
  analogWrite(RPWM2, LOW);      // 모터2 후진 속도 없음
  
}

// 후진
void moveBackward(int speed) {
  analogWrite(LPWM1, LOW);      // 모터1 전진 속도 없음
  analogWrite(RPWM1, speed);    // 모터1 후진 속도 설정
  analogWrite(LPWM2, LOW);      // 모터2 전진 속도 없음
  analogWrite(RPWM2, speed+27); // 모터2 후진 속도 설정
}

// 좌회전
void turnLeft(int speed) {
  analogWrite(LPWM1, LOW);      // 모터1 전진 속도 없음
  analogWrite(RPWM1, speed);    // 모터1 후진 속도 설정
  analogWrite(LPWM2, speed+27); // 모터2 전진 속도 설정
  analogWrite(RPWM2, LOW);      // 모터2 후진 속도 없음
}

// 우회전
void turnRight(int speed) {
  analogWrite(LPWM1, speed);    // 모터1 전진 속도 설정
  analogWrite(RPWM1, LOW);      // 모터1 후진 속도 없음
  analogWrite(LPWM2, LOW);      // 모터2 전진 속도 없음
  analogWrite(RPWM2, speed+27); // 모터2 후진 속도 설정
}

// 정지
void stopMotors() {
  analogWrite(LPWM1, LOW);   // 모터1 전진 속도 없음
  analogWrite(RPWM1, LOW);   // 모터1 후진 속도 없음
  analogWrite(LPWM2, LOW);   // 모터2 전진 속도 없음
  analogWrite(RPWM2, LOW);   // 모터2 후진 속도 없음
}

// 전진 + 좌회전
void forwardLeft(int speed, int turnSpeed) {
  analogWrite(LPWM1, speed - turnSpeed);     // 오른쪽 바퀴 느리게 전진
  analogWrite(RPWM1, LOW);              
  analogWrite(LPWM2, speed + 27);            // 왼쪽 바퀴 빠르게 전진
  analogWrite(RPWM2, LOW);
}

// 전진 + 우회전
void forwardRight(int speed, int turnSpeed) {
  analogWrite(LPWM1, speed);                  // 오른쪽 바퀴 빠르게 전진
  analogWrite(RPWM1, LOW);              
  analogWrite(LPWM2, speed + 27 - turnSpeed); // 왼쪽 바퀴 느리게 전진
  analogWrite(RPWM2, LOW);
}

// 후진 + 좌회전
void backwardLeft(int speed, int turnSpeed) {
  analogWrite(LPWM1, LOW);
  analogWrite(RPWM1, speed - turnSpeed);     // 오른쪽 바퀴 느리게 후진
  analogWrite(LPWM2, LOW);
  analogWrite(RPWM2, speed + 27);            // 왼쪽 바퀴 빠르게 후진
}

// 후진 + 우회전
void backwardRight(int speed, int turnSpeed) {
  analogWrite(LPWM1, LOW);
  analogWrite(RPWM1, speed);                  // 오른쪽 바퀴 빠르게 후진
  analogWrite(LPWM2, LOW);
  analogWrite(RPWM2, speed + 27 - turnSpeed); // 왼쪽 바퀴 느리게 후진
}