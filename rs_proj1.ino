#include <Servo.h>

const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 6;

Servo myServo;

long duration;
float distance;
int currentDirection = 90;
bool turnedAway = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myServo.write(currentDirection);
  delay(1000);
}

void loop() {
  distance = getDistance();

  // Send data in structured format
  Serial.print("distance:");
  Serial.print(distance);
  Serial.print(",angle:");
  Serial.println(currentDirection);

  if (!turnedAway && distance > 0 && distance < 15) {
    currentDirection = (currentDirection == 90) ? 0 : 180;
    myServo.write(currentDirection);
    turnedAway = true;
    delay(1000);
  } 
  else if (turnedAway && distance > 0 && distance < 10) {
    currentDirection = (currentDirection == 0) ? 180 : 0;
    myServo.write(currentDirection);
    delay(1000);
  }

  delay(500);
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}
