#ifndef CONSTANTS_h
#define CONSTANTS_h



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



////////////////////////////////////////////////////////////////////// Geometric Measurements //////////////////////////////////////////////////////////////////////





#endif // CONSTANTS_h