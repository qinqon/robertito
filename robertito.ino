
#include <Servo.h>

Servo rearRightServo;
Servo frontRightServo;
Servo rearLeftServo;
Servo frontLeftServo;

const int maxSpeed = 255;

const int dirRightPin = 18;
const int pwmRightPin =19;
const int dirLeftPin = 10;
const int pwmLeftPin = 9;
const int ledPin = 13;

const int frontRightServoPin = 14;
const int rearRightServoPin = 13; 
const int frontLeftServoPin = 11;
const int rearLeftServoPin = 12;

const int frontRightServoResetPos = 93;
const int rearRightServoResetPos = 94;
const int rearLeftServoResetPos = 91;
const int frontLeftServoResetPos = 94;

const int maxTurnAngle = 30;
const float turnAngleFactor = 0.5;

void setup() {
   Serial.begin(9600);
   Serial2.begin(9600);
   pinMode(dirRightPin, OUTPUT);
   pinMode(dirLeftPin, OUTPUT);
   rearRightServo.attach(rearRightServoPin);
   frontRightServo.attach(frontRightServoPin);
   rearLeftServo.attach(rearLeftServoPin);
   frontLeftServo.attach(frontLeftServoPin);
/*
  straight();
  delay(2000);
  left();
  delay(2000);
  right();
  delay(2000);
  straight();
  delay(2000);
  */
}

int speed = 0;
int control = 0;
int dirLeft = LOW;
int dirRight = LOW;

void straight() {
 
  rearLeftServo.write(rearLeftServoResetPos);
  frontLeftServo.write(frontLeftServoResetPos);
  frontRightServo.write(frontRightServoResetPos);
  rearRightServo.write(rearRightServoResetPos);
}

void left() {
  frontLeftServo.write(frontLeftServoResetPos + maxTurnAngle);
  rearLeftServo.write(rearLeftServoResetPos - maxTurnAngle);
  frontRightServo.write(frontRightServoResetPos - maxTurnAngle * turnAngleFactor);
  rearRightServo.write(rearRightServoResetPos + maxTurnAngle * turnAngleFactor);
}

void right() {
  frontLeftServo.write(frontLeftServoResetPos - maxTurnAngle * turnAngleFactor);
  rearLeftServo.write(rearLeftServoResetPos + maxTurnAngle * turnAngleFactor);
  frontRightServo.write(frontRightServoResetPos + maxTurnAngle);
  rearRightServo.write(rearRightServoResetPos - maxTurnAngle);
}

void turn() {
  frontLeftServo.write(frontLeftServoResetPos - maxTurnAngle);
  rearLeftServo.write(rearLeftServoResetPos + maxTurnAngle);
  frontRightServo.write(frontRightServoResetPos - maxTurnAngle);
  rearRightServo.write(rearRightServoResetPos + maxTurnAngle);
}

void loop() {
  if(Serial2.available() > 0){ // Checks whether data is comming from the serial port
    control = Serial2.read(); // Reads the data from the serial port
    Serial.println(control);
    if (control == '0') {
      Serial.println("STOP"); // Send back, to the phone, the String "LED: ON"
      straight();
      speed = 0;
      if (dirLeft != dirRight) {
        dirRight = HIGH;
        dirLeft = HIGH;
      }
    }
    else if (control == '1') {
      Serial.println("UP");
      speed = maxSpeed;
      dirLeft = LOW;
      dirRight = LOW;
      straight();
    } 
    else if (control == '2') {
      Serial.println("DOWN");
      speed = maxSpeed;
      dirLeft = HIGH;
      dirRight = HIGH;
      straight();
    }
    else if (control == '3') {
      Serial.println("LEFT");
      speed = maxSpeed;
      left();
      
    }
    else if (control == '4') {
      Serial.println("RIGHT");
      speed = maxSpeed;
      right();
    }
    else if (control == '5') {
      Serial.println("STRAIGTH");
      straight();
    }
    else if (control == '6') {
      Serial.println("TURN LEFT");
      dirLeft = HIGH;
      dirRight = LOW;
      speed = maxSpeed;
      turn();
    }
    else if (control == '7') {
      Serial.println("TURN RIGHT");
      dirLeft = LOW;
      dirRight = HIGH;
      speed = maxSpeed;
      turn();
    }
    analogWrite(pwmLeftPin, speed);
    digitalWrite(dirLeftPin, dirLeft);
    analogWrite(pwmRightPin, speed);
    digitalWrite(dirRightPin, dirRight);
    
  }



}
