//모터 입력 실험

#include <SoftwareSerial.h>

// HC-06 모듈의 TX, RX 핀 정의
SoftwareSerial BTSerial(2, 3);
// SoftwareSerial 객체 생성

// 디지털 핀 선언
const int leftMotorForwardPin = 5;
const int rightMotorForwardPin = 6;
const int buttonPin = 7;

int actualWaypoints = 0;
float (*waypoints)[2];
int waypointIndex = 0;

//차량 위치 초기값 설정
float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;
float goalX = 0.0;
float goalY = 0.0;
int currentWaypointIndex = 0;  // 경로 추적에 사용할 현재 웨이포인트 인덱스
float dx = goalX - robotX;
float dy = goalY - robotY;
float distance = sqrt(dx * dx + dy * dy);

//y축 해상
const int IMAGE_HEIGHT = 480;

//경로추종 끝났는지 확
bool isPathFollowingComplete = false;

//경로 수신, 경로 추종 모드 전환 변수 선언
bool isStarted = false;
bool isWaypointDataReceived = false;

//핀 모드, 시리얼 통신 설정
void setup() {
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
  BTSerial.begin(9600);
}

//버튼 디바운싱 변수 선언
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool lastButtonState = HIGH;
bool buttonState;

void loop() {
  bool reading = digitalRead(buttonPin);  //버튼 상태 확인 및 저장

  if (reading != lastButtonState) {  //버튼 상태 변경 확인
    lastDebounceTime = millis();     //버튼 눌린 시간 저장
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {  //디바운싱 시간동안 대기
    if (reading != buttonState) {                       //실제로 버튼 상태 변경
      buttonState = reading;                            //버튼 상태 변경된 것으로 업데이트

      if (buttonState == LOW) {
        isStarted = !isStarted;
      }
    }
  }

  lastButtonState = reading;

  if (!isStarted && !isWaypointDataReceived) {
    receiveWaypointData();
  } else if (isStarted) {
    receivePositionData();
    followPath();


    if (isPathFollowingComplete) {
      return;
    }
  }
  delay(50);
}

void receiveWaypointData() {
  if (BTSerial.available()) {
    String data = BTSerial.readStringUntil(';');
    if (data.startsWith("W")) {
      parseWaypoint(data.substring(1));
    } else if (data.startsWith("S")) {
      actualWaypoints = data.substring(1).toInt();
      waypoints = new float[actualWaypoints][2];
    }
  
  }
}

void receivePositionData() {
  for(int i = 0; i < 3; i++){
  if (BTSerial.available()) {
    String data = BTSerial.readStringUntil(';');
    if (data.startsWith("C")) {
      parseCurrentPosition(data.substring(1));
    } else if (data.startsWith("A")) {
      parseAngle(data.substring(1));
    }
  }
  }
}

void followPath() {

  goalX = waypoints[currentWaypointIndex][0];
  goalY = waypoints[currentWaypointIndex][1];

  dx = goalX - robotX;
  dy = goalY - robotY;
  distance = sqrt(dx * dx + dy * dy);

  for (int i = 0; i < 8; i++) {
    if (distance < 100 && currentWaypointIndex < actualWaypoints - 1) {
      currentWaypointIndex++;

      goalX = waypoints[currentWaypointIndex][0];
      goalY = waypoints[currentWaypointIndex][1];
      dx = goalX - robotX;
      dy = goalY - robotY;
      distance = sqrt(dx * dx + dy * dy);
    } else if (currentWaypointIndex == actualWaypoints - 1 && distance > 60) {
      goalX = waypoints[currentWaypointIndex][0];
      goalY = waypoints[currentWaypointIndex][1];
      dx = goalX - robotX;
      dy = goalY - robotY;
      distance = sqrt(dx * dx + dy * dy);
    } else if (currentWaypointIndex == actualWaypoints - 1 && distance < 60) {
      while(1) {
        analogWrite(leftMotorForwardPin, 0);
        analogWrite(rightMotorForwardPin, 0);
     }
    }
  }

  float goalAngle = atan2(dy, dx);
  float alpha = goalAngle - robotTheta;
  if (alpha > PI) {
    alpha -= 2 * PI;
  } else if (alpha < -PI) {
    alpha += 2 * PI;
  }

  float L = 95;
  float Lw = 100;

  float ld = 100;

  float beta = atan2(2 * L * sin(alpha) / ld, 1);

  float maxBeta = PI / 2.5;
  if (beta > maxBeta) {
    beta = maxBeta;
  } else if (beta < -maxBeta) {
    beta = -maxBeta;
  }

  float v = 70.0;
  float w = v * tan(beta) / L;

  float leftSpeed = (v - w * (Lw / 2)) * (0.5 / v);
  float rightSpeed = (v + w * (Lw / 2)) * (0.5 / v);

  leftSpeed = max(0, min(leftSpeed, 1));
  rightSpeed = max(0, min(rightSpeed, 1));

  analogWrite(leftMotorForwardPin, leftSpeed * 255);
  analogWrite(rightMotorForwardPin, rightSpeed * 255);


  /*Serial.print("Left: ");
  Serial.print(leftSpeed);
  Serial.print(" Right: ");
  Serial.println(rightSpeed);

  Serial.print("RobotX: ");
  Serial.print(robotX);
  Serial.print(" RobotY: ");
  Serial.println(robotY);

  Serial.print(", goalX: ");
  Serial.print(goalX);
  Serial.print(" goalY: ");
  Serial.println(goalY);
  Serial.print(" theta: ");
  Serial.println(robotTheta);
  Serial.print(" lastgoalX: ");
  Serial.print(waypoints[actualWaypoints - 1][0]);
  Serial.print(" lastgoalY: ");
  Serial.println(waypoints[actualWaypoints - 1][1]);*/
  delay(100);
}

void parseCurrentPosition(String data) {
  int commaIndex = data.indexOf(',');
  if (commaIndex == -1) return;

  robotX = data.substring(0, commaIndex).toFloat();
  robotY = IMAGE_HEIGHT - data.substring(commaIndex + 1).toFloat();  // y값 반전
}

void parseAngle(String data) {
  robotTheta = data.toFloat();
  robotTheta = robotTheta;  // 라디안 단위로 계산

  // 각도를 -PI ~ PI 범위로 정규화
  if (robotTheta > PI) {
    robotTheta -= 2 * PI;
  } else if (robotTheta < -PI) {
    robotTheta += 2 * PI;
  }
}

void parseWaypoint(String data) {
  int commaIndex = data.indexOf(',');
  if (commaIndex == -1) return;

  float x = data.substring(0, commaIndex).toFloat();
  float y = IMAGE_HEIGHT - data.substring(commaIndex + 1).toFloat();  // y값 반전
  if (waypointIndex < actualWaypoints) {
    waypoints[waypointIndex][0] = x;
    waypoints[waypointIndex][1] = y;
    // 배열에 저장된 웨이포인트 값 출력
    Serial.print("Waypoint ");
    Serial.print(waypointIndex);
    Serial.print(": x = ");
    Serial.print(x);
    Serial.print(", y = ");
    Serial.println(y);

    waypointIndex++;

    // 배열이 꽉 찼을 경우, 모든 웨이포인트 출력
    if (waypointIndex == actualWaypoints) {
      Serial.println("All Waypoints:");
      for (int i = 0; i < actualWaypoints; i++) {
        Serial.print("Waypoint ");
        Serial.print(i);
        Serial.print(": x = ");
        Serial.print(waypoints[i][0]);
        Serial.print(", y = ");
        Serial.println(waypoints[i][1]);
      }
    }
    
    
  }
}
