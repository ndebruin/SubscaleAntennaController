#ifndef CONSTANTS_h
#define CONSTANTS_h

#include <BasicLinearAlgebra.h>

struct AnglePose{
    double theta;
    double phi;
};

// x, y, z
using Coord3d = BLA::Matrix<3>;

struct CableLengths{
    double cable1;
    double cable2;
    double cable3;
};

////////////////////////////////////////////////////////////////////// Positional Sensor //////////////////////////////////////////////////////////////////////

#define xAxisPin 5
#define yAxisPin 4

#define xAxisMin -45.0
#define xAxisMax 45.0

#define yAxisMin -45.0
#define yAxisMax 45.0

// we are assuming a linear relationship, which is a valid assumption given the used sensor

// NEED TO CALIBRATE
#define xAxisMinValue 0
#define xAxisMaxValue 1024

#define yAxisMinValue 0
#define yAxisMaxValue 1024

constexpr double xAxisScalar = ((double)(xAxisMax - xAxisMin) / (double)(xAxisMaxValue - xAxisMinValue));
constexpr double yAxisScalar = ((double)(yAxisMax - yAxisMin) / (double)(yAxisMaxValue - yAxisMinValue));


////////////////////////////////////////////////////////////////////// Actuators //////////////////////////////////////////////////////////////////////

#define actuator1Channel 1
#define actuator2Channel 2
#define actuator3Channel 3

#define minAngle 0.0 // degrees
#define minPulse 500 // microseconds

#define maxAngle 270.0 // degrees
#define maxPulse 2500 // microseconds

#define channel1Default // microseconds
#define channel2Default // microseconds
#define channel3Default // microseconds



////////////////////////////////////////////////////////////////////// Geometric Measurements //////////////////////////////////////////////////////////////////////

// r = 0.701 in
// R = 4.783 in
// l = 0.875in
// h = 0.181 in UP from rotation point

#define inToMM 25.4
#define mmToIn 1/25.4

// preconverted to MM
#define attachmentRadius 17.8054
#define pulleyRadius 121.4882
#define feedPointDistance 22.225
#define pulleyHeight 4.5974

// angles for each of the attachment / pulley points
// 0, 120, 240 degrees converted to radians
#define Beta1 0.0
#define Beta2 2 * PI / 3
#define Beta3 4 * PI / 3

#define servoPulleyRadius 8.5 // mm

// we are placing our origin for the 'global' frame (that being the frame of the overall dish) at the rotation point

Coord3d pulleyPoint1 = {pulleyRadius * cos(Beta1), pulleyRadius * sin(Beta1), pulleyHeight};
Coord3d pulleyPoint2 = {pulleyRadius * cos(Beta2), pulleyRadius * sin(Beta2), pulleyHeight};
Coord3d pulleyPoint3 = {pulleyRadius * cos(Beta3), pulleyRadius * sin(Beta3), pulleyHeight};

// note that while these are in the 'body' frame (that being the frame of the horn)
// while they conveniently are also the coordinates in the global frame in the case where the horn is pointing straight down ('home' position), this is not always the case
Coord3d attachmentPoint1 = {attachmentRadius * cos(Beta1), attachmentRadius * sin(Beta1), -feedPointDistance};
Coord3d attachmentPoint2 = {attachmentRadius * cos(Beta2), attachmentRadius * sin(Beta2), -feedPointDistance};
Coord3d attachmentPoint3 = {attachmentRadius * cos(Beta3), attachmentRadius * sin(Beta3), -feedPointDistance};

#define defaultLength1
#define defaultLength2
#define defaultLength3
 

#endif // CONSTANTS_h