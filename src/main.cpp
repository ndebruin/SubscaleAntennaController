#include <Arduino.h>
#include <Alfredo_NoU2.h>

#include "JoystickSensor.hpp"

JoystickSensor positionSensor = JoystickSensor();




void setup() {
  positionSensor.begin();

}

void loop() {
  positionSensor.update();

}
