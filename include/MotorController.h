#ifndef MOTORCONTROLLER_h
#define MOTORCONTROLLER_h

#include <Arduino.h>

#include <Alfredo_NoU2.h>

#include "Constants.h"


class MotorController
{
    public:
        MotorController(NoU_Servo* Servo) : motor(Servo)
        {
            motor->setMinimumPulse(minPulse);
            motor->setMaximumPulse(maxPulse);
        }

        void setWhichMotor(size_t index){
            switch (index){
                case 1:
                    defaultLength = defaultLength1;
                    defaultPos = channel1Default;
                    break;
                case 2:
                    defaultLength = defaultLength2;
                    defaultPos = channel2Default;
                    break;
                case 3:
                    defaultLength = defaultLength3;
                    defaultPos = channel3Default;
                    break;
            }
            defaultAngle = microSecondsToAngle(defaultPos);
        }

        double getCurrentLength(){ return currentLength; }

        void setLength(double desiredLength)
        {
            currentLength = desiredLength;
            currentAngle = cableLengthToAngle(currentLength);
            motor->writeMicroseconds(angleToMicroseconds(currentAngle));
        }

    private:
        NoU_Servo* motor;

        double defaultLength;
        double defaultPos;
        double defaultAngle;

        double currentAngle; // stored in degrees
        double currentLength; // mm

        double microSecondsToAngle(double microseconds){ // degrees
            return dmap(microseconds, minPulse, maxPulse, minAngle, maxAngle);
        }

        double angleToMicroseconds(double angle){ // degrees
            return dmap(angle, minAngle, maxAngle, minPulse, maxPulse);
        }

        double angleToCableLength(double angle){ // degrees
            // find the angular distance between the provided angle and the default angle
            double angularDiff = (angle - defaultAngle) * DEG_TO_RAD;

            // convert angular diff to linear distance
            double linearDiff = angularDiff * servoPulleyRadius;

            // add that difference to the default length, to get the final length
            return linearDiff + defaultLength;
        }

        double cableLengthToAngle(double desiredLength){
            // subtract the default cable length
            double linearDiff = desiredLength - defaultLength;
            
            // convert linear diff to angular diff
            double angularDiff = linearDiff / servoPulleyRadius;

            return (RAD_TO_DEG * angularDiff) + defaultAngle;
        }

        double dmap(double val, double in_min, double in_max, double out_min, double out_max) {
            return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
        
};


#endif // MOTORCONTROLLER_h
