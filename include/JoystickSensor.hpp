#ifndef JOYSTICKSENSOR_h
#define JOYSTICKSENSOR_h

#include <Arduino.h>

#include "PositionalSensor.h"
#include "Constants.h"


class JoystickSensor : public PositionalSensor
{
    public:
        JoystickSensor(){}

        uint8_t begin() override
        {
            analogReadResolution(12);

            if(adcAttachPin(xAxisPin) && adcAttachPin(yAxisPin)){
                return 0; // all went well
            }
            return 1; // something has gone wrong
        }

        uint8_t update() override
        {
            currentPose.theta = getRawTheta() * xAxisScalar;
            currentPose.phi = getRawPhi() * yAxisScalar;

            return 0;
        }

        double getRawTheta() override { return (double)analogRead(xAxisPin); }

        double getRawPhi() override { return (double)analogRead(yAxisPin); }
};


#endif // JOYSTICKSENSOR_h
