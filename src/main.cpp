#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "JoystickSensor.hpp"

#include "InverseKinematicsEngine.hpp"

#include "MotionController.hpp"
#include "MotorController.h"

JoystickSensor positionSensor = JoystickSensor();

NoU_Servo servo1(actuator1Channel);
NoU_Servo servo2(actuator2Channel);
NoU_Servo servo3(actuator3Channel);

MotorController motor1(&servo1);
MotorController motor2(&servo2);
MotorController motor3(&servo3);

MotionController motionController(&motor1, &motor2, &motor3, &positionSensor);


void setup() {
  Serial.begin(9600);
  positionSensor.begin();

}

void loop() {
  positionSensor.update();

}
