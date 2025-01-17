#ifndef MOTIONCONTROLLER_h
#define MOTIONCONTROLLER_h

#include <Arduino.h>

#include "Constants.h"
#include "MotorController.h"

#include "JoystickSensor.hpp"

#include "InverseKinematicsEngine.hpp"


class MotionController
{
    public:
        MotionController(MotorController* Motor1, MotorController* Motor2, MotorController* Motor3, JoystickSensor* sensor) : motor1(Motor1), motor2(Motor2), motor3(Motor3)
        { }

        void update()
        {
            sensor->update();
        }

        void setDesiredPosition(AnglePose desiredPose)
        {
            // figure out which way we need to move in comparison to the existing one
            CableLengths desiredCableLengths = InverseKinematicsEngine::getDesiredCableLengths(desiredPose);

           // go through and see which cables are getting longer vs shorter. the ones getting longer should run first
           CableLengths currentLengths = {motor1->getCurrentLength(), motor2->getCurrentLength(), motor3->getCurrentLength()};
           CableLengths diff = cableLengthSubtract(desiredCableLengths, currentLengths);

           bool move1First = false;
           bool move2First = false;
           bool move3First = false;

           if(diff.cable1 > diff.cable2 > diff.cable3){
            motor1->setLength(desiredCableLengths.cable1);
            motor2->setLength(desiredCableLengths.cable2);
            motor3->setLength(desiredCableLengths.cable3);
           }
           else if(diff.cable1 > diff.cable3 > diff.cable2){
            motor1->setLength(desiredCableLengths.cable1);
            motor3->setLength(desiredCableLengths.cable3);
            motor2->setLength(desiredCableLengths.cable2);
           }
           else if(diff.cable2 > diff.cable1 > diff.cable3){
            motor2->setLength(desiredCableLengths.cable2);
            motor1->setLength(desiredCableLengths.cable1);
            motor3->setLength(desiredCableLengths.cable3);
           }
           else if(diff.cable2 > diff.cable3 > diff.cable1){
            motor2->setLength(desiredCableLengths.cable2);
            motor3->setLength(desiredCableLengths.cable3);
            motor1->setLength(desiredCableLengths.cable1);
           }
           else if(diff.cable3 > diff.cable1 > diff.cable2){
            motor3->setLength(desiredCableLengths.cable3);
            motor1->setLength(desiredCableLengths.cable1);
            motor2->setLength(desiredCableLengths.cable2);            
           }
           else if(diff.cable3 > diff.cable2 > diff.cable1){
            motor3->setLength(desiredCableLengths.cable3);
            motor2->setLength(desiredCableLengths.cable2);
            motor1->setLength(desiredCableLengths.cable1);
           }
        }

    private:
        MotorController* motor1;
        MotorController* motor2;
        MotorController* motor3;
        JoystickSensor* sensor;


        AnglePose anglePoseSubtraction(AnglePose pose1, AnglePose pose2)
        {
            AnglePose diff;
            diff.theta = pose1.theta - pose2.theta;
            diff.phi = pose1.phi - pose2.phi;
            return diff;
        }

        // returns CCW+ theta from the anglePose (direction vector)
        double findDirection(AnglePose poseDirection)
        {
            // turn the angles into [-1,1] (like on a unit circle)
            double x = poseDirection.theta / 90.0;
            double y = poseDirection.phi / 90.0;

            double angle = atan2(y, x);
            return angle;
        }

        CableLengths cableLengthSubtract(CableLengths cables1, CableLengths cables2)
        {
            CableLengths diff;
            diff.cable1 = cables1.cable1 - cables2.cable1;
            diff.cable2 = cables1.cable2 - cables2.cable2;
            diff.cable3 = cables1.cable3 - cables2.cable3;
            return diff;
        }
};


#endif // MOTIONCONTROLLER_h
