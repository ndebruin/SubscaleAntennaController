#ifndef POSITIONALSENSOR_h
#define POSITIONALSENSOR_h

#include <Arduino.h>

#include "Constants.h"

class PositionalSensor
{
    public:
        PositionalSensor(){}

        virtual uint8_t begin() = 0;
        virtual uint8_t update() = 0;

        double getTheta(){ return currentPose.theta; }
        double getPhi(){return currentPose.phi; }

        double getX(){ return getTheta(); }
        double getY(){ return getPhi(); }

        AnglePose getAngularPose(){ return currentPose; }

        virtual double getRawTheta() = 0;
        virtual double getRawPhi() = 0;
    protected:
        AnglePose currentPose;
};


#endif // POSITIONALSENSOR_h
