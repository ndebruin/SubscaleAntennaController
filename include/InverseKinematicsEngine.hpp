#ifndef INVERSEKINEMATICS_h
#define INVERSEKINEMATICS_h

#include <Arduino.h>

#include "Constants.h"


class InverseKinematicsEngine
{
    public:
        InverseKinematicsEngine();


    private:
        Coord3d bodyToGlobalTransform(Coord3d bodyFrameCoord, AnglePose rotations)
        {
            double theta = rotations.theta * DEG_TO_RAD;
            double phi = rotations.phi * DEG_TO_RAD;
            BLA::Matrix<3,3> rotationMatrix = 
            {
                cos(theta)*cos(phi), -sin(phi), sin(theta)*cos(phi),
                cos(theta)*sin(phi), cos(phi), sin(theta)*sin(phi),
                -sin(theta), 0, cos(theta)
            };

            Coord3d inGlobalFrame = rotationMatrix * bodyFrameCoord;
            return inGlobalFrame;
        }
        Coord3d globalToBodyTransform(Coord3d bodyFrameCoord, AnglePose rotations)
        {
            double theta = rotations.theta * DEG_TO_RAD;
            double phi = rotations.phi * DEG_TO_RAD;
            BLA::Matrix<3,3> rotationMatrix = 
            {
                cos(theta)*cos(phi), -sin(phi), sin(theta)*cos(phi),
                cos(theta)*sin(phi), cos(phi), sin(theta)*sin(phi),
                -sin(theta), 0, cos(theta)
            };

            Coord3d inGlobalFrame = Inverse(rotationMatrix) * bodyFrameCoord;
            return inGlobalFrame;
        }
        
};


#endif // INVERSEKINEMATICS_h
