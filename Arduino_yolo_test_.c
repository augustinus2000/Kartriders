#include <SoftwareSerial.h>

SoftwareSerial HC_06(12, 13); // 소프트웨어 시리얼 사용
// 아두이노 RX = 12번, 아두이노 TX = 13번 -->> 블루투스 모듈 TX를 아두이노 12번에, 
//                                            블루투스 모듈 RX를 아두이노 13번에 넣기

// BTS7960 핀
const int LPWM1 = 5;  // 왼쪽 PWM 입력 (전진)
const int RPWM1 = 6;  // 오른쪽 PWM 입력 (후진)
const int LEN1 = 2;   // 왼쪽 모터 활성화
const int REN1 = 4;   // 오른쪽 모터 활성화

const int LPWM2 = 9;  // 왼쪽 PWM 입력 (전진)
const int RPWM2 = 10; // 오른쪽 PWM 입력 (후진)
const int LEN2 = 7;   // 왼쪽 모터 활성화
const int REN2 = 8;   // 오른쪽 모터 활성화

/*
모터 드라이버의 LEN과 REN 신호는 항상 1에 물려두고 써도 되지만, 
쓸 때는 1로, 안 쓸 때는 0으로 두면 배터리를 절약할 수 있어요.
*/

void setup() {
  // 핀 설정
  pinMode(LPWM1, OUTPUT); // 핀 모드는 OUTPUT로 설정
  pinMode(RPWM1, OUTPUT);
  pinMode(LEN1, OUTPUT);
  pinMode(REN1, OUTPUT);

  pinMode(LPWM2, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LEN2, OUTPUT);
  pinMode(REN2, OUTPUT);

  // BTS7960 모터 구동부 활성화 핀 초기화
  digitalWrite(LEN1, LOW); // 아두이노의 디지털 핀에 HIGH 또는 LOW 값을 설정하기 위해 사용
  digitalWrite(REN1, LOW); // 보기 편한 거 같아서 0과 1 대신 HIGH와 LOW 사용했습니다.
  digitalWrite(LEN2, LOW);
  digitalWrite(REN2, LOW);

  // 시리얼 통신 초기화
  //Serial.begin(9600); // 시리얼 모니터와 통신, 아두이노 IDE에서 볼 수 있는 UI창.
                      // 노트북과 아두이노를 usb로 연결해놔야 볼 수 있는데,
                      // 고장 원인이 파악이 안되어서 usb 연결은 안함. 없어도 되는 코드

  HC_06.begin(9600);    // 블루투스 모듈과 통신
  // Serial.println("블루투스 준비!");
}

void loop() {
  if (HC_06.available()) {
    String command = HC_06.readStringUntil('\n');  // '\n'으로 끝나는 문자열 읽기
    
    int commaIndex = command.indexOf(',');  // 쉼표 위치 찾기 (python 코드에서 쉼표로 구분 했음)
    if (commaIndex != -1) { // 쉼표 위치가 찾아졌으면
      String charSignal = command.substring(0, commaIndex);  // 쉼표 앞에 있는 문자
      String numSignal = command.substring(commaIndex + 1);  // 쉼표 뒤에 있는 숫자
      
      char receivedChar = charSignal.charAt(0);  // 문자로 변환
      int receivedNum = numSignal.toInt();  // 숫자로 변환

      // Serial.print("받은 문자: "); // 시리얼 모니터를 쓴다면 이 문장이 출력됩니다.
      //Serial.println(receivedChar);
      // Serial.print("받은 숫자: ");
      // Serial.println(receivedNum);

      // 문자에 따라 모터 제어
      if (receivedChar == 'f') {  // 전진
        // Serial.println("전진");   // 마찬가지로 시리얼 모니터를 쓴다면 이 문장이 출력돼요.
        moveForward(receivedNum);  // speed는 받은 숫자로 설정 -> pwm 값을 받았다!
      }
      else if (receivedChar == 'b') {  // 후진
        // Serial.println("후진");
        moveBackward(receivedNum);
      }
      else if (receivedChar == 'l') {  // 좌회전
        // Serial.println("좌회전");
        turnLeft(receivedNum);
      }
      else if (receivedChar == 'r') {  // 우회전
        // Serial.println("우회전");
        turnRight(receivedNum);
      }
      else if (receivedChar == 's') {  // 정지
        // Serial.println("정지");
        stopMotors();
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
