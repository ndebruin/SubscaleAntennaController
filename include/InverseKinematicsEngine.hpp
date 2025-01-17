#ifndef INVERSEKINEMATICS_h
#define INVERSEKINEMATICS_h

#include <Arduino.h>

#include "Constants.h"


class InverseKinematicsEngine
{
    public:
        static CableLengths getDesiredCableLengths(AnglePose desiredRotation)
        {
            CableLengths idealCableLengths;
            // see PDF in the repo for the actual derivation of this.

            // going to calculate first for cable 1

            // find the position in the global frame of the attachment point on the horn when the desired rotation is applied.
            Coord3d attachmentPoint1Global = bodyToGlobalTransform(attachmentPoint1, desiredRotation);

            // now we find the translation vector from the pulley point on the rim of the dish to the attachment point on the horn.
            Coord3d translationVector1 = attachmentPoint1Global - pulleyPoint1; // thanks BLA for doing this math for us

            // now we take the magnitude of this vector, as that gives us the euclidian distance in 3d space, which is equivalent to the length of an ideal tensioned cable.
            // this assumes that the cable is perfectly tensioned, and perfectly follows the hypothetical euclidian distance line.
            // this assumption is likely incorrect in practice, but that's the advantage of directly measuring the angle of the horn. 
            // We can correct for this imperfect assumption with a feedback loop in the motion controller.

            idealCableLengths.cable1 = magnitude(translationVector1);

            // we can now repeat all of this for cables 2 & 3

            Coord3d attachmentPoint2Global = bodyToGlobalTransform(attachmentPoint2, desiredRotation);
            Coord3d translationVector2 = attachmentPoint2Global - pulleyPoint2;
            idealCableLengths.cable2 = magnitude(translationVector2);

            Coord3d attachmentPoint3Global = bodyToGlobalTransform(attachmentPoint3, desiredRotation);
            Coord3d translationVector3 = attachmentPoint3Global - pulleyPoint3;
            idealCableLengths.cable3 = magnitude(translationVector3);

            return idealCableLengths;
        }

    // private:
        static Coord3d bodyToGlobalTransform(Coord3d bodyFrameCoord, AnglePose rotations)
        {
            double theta = rotations.theta * DEG_TO_RAD;
            double phi = rotations.phi * DEG_TO_RAD;
            BLA::Matrix<3,3> rotationMatrix = // see PDF in repo for where this rotation matrix comes from
            {
                cos(theta)*cos(phi), -sin(phi), sin(theta)*cos(phi),
                cos(theta)*sin(phi), cos(phi), sin(theta)*sin(phi),
                -sin(theta), 0, cos(theta)
            };

            Coord3d inGlobalFrame = rotationMatrix * bodyFrameCoord;
            return inGlobalFrame;
        }
        static Coord3d globalToBodyTransform(Coord3d bodyFrameCoord, AnglePose rotations)
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

        // unforunately, BLA does not have a built in magnitude function
        static double magnitude(Coord3d coord)
        {
            return sqrt(sq(coord(0)) + sq(coord(1)) + sq(coord(2)));
        }
        
};


#endif // INVERSEKINEMATICS_h
