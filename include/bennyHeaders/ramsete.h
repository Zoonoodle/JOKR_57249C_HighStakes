#ifndef _ROBOT_CONTROL_H_
#define _ROBOT_CONTROL_H_

#include <cmath>
#include <cstdio>
#include "main.h"

// Define a Pose structure (x, y in inches, theta in radians)
struct Pose {
    double x;
    double y;
    double theta;  // in radians
};

// Function declarations
double sinc(double x);
void ramseteController(const Pose &current, const Pose &desired,
                       double desiredV, double desiredOmega,
                       double &cmdV, double &cmdOmega);
void diffDriveKinematics(double cmdV, double cmdOmega, double trackWidth,
                         double &leftVel, double &rightVel);
double inchesPerSecToRPM(double inchesPerSec, double wheelDiameter);
void runRamseteControlStep(const Pose &currentPose, const Pose &desiredPose,
                           double desiredV, double desiredOmega,
                           double trackWidth, double wheelDiameter,
                           double &leftMotorRPM, double &rightMotorRPM);

// RAMSETE gains
extern const double b;
extern const double zeta;

#endif // _ROBOT_CONTROL_H_
