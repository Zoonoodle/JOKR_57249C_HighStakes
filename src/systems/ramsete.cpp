#include <cmath>
#include <cstdio>


const double TRACK_WIDTH = 11.0;  // Robot's track width in inches
const double WHEEL_DIAMETER = 2.75; // Wheel diameter in inches
const double MAX_RPM = 450.0; // Drivetrain maximum RPM
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
const double MAX_VEL_IPS = (MAX_RPM / 60.0) * WHEEL_CIRCUMFERENCE; // Max inches per second
const double b = 2.0;  // RAMSETE Aggressiveness
const double zeta = 0.7;  // RAMSETE Damping


struct Pose {
    double x, y, theta;  // theta in radians
};

struct Velocity {
    double linear, angular;
};

/**
 * @brief RAMSETE Controller (from WPILib)
 * @param current Current pose (x, y, theta in radians)
 * @param desired Desired pose (x, y, theta in radians)
 * @param desiredVel Desired linear velocity (inches/sec)
 * @param desiredOmega Desired angular velocity (radians/sec)
 * @return Velocity Commanded velocity (linear, angular)
 */
Velocity ramseteController(Pose current, Pose desired, double desiredVel, double desiredOmega) {
    double errorX = cos(current.theta) * (desired.x - current.x) + sin(current.theta) * (desired.y - current.y);
    double errorY = -sin(current.theta) * (desired.x - current.x) + cos(current.theta) * (desired.y - current.y);
    double errorTheta = desired.theta - current.theta;
    errorTheta = atan2(sin(errorTheta), cos(errorTheta)); // Normalize to [-pi, pi]

    // Gain schedule based on velocity and curvature
    double k = 2 * zeta * sqrt(desiredOmega * desiredOmega + b * desiredVel * desiredVel);

    // Compute control signals
    double cmdV = desiredVel * cos(errorTheta) + k * errorX;
    double cmdOmega = desiredOmega + b * desiredVel * (sin(errorTheta) / errorTheta) * errorY + k * errorTheta;

    return {cmdV, cmdOmega};
}


struct WheelSpeeds {
    double left, right;
};

/**
 * @brief Converts linear and angular velocity to left/right wheel speeds
 * @param linearVel Linear velocity (inches per second)
 * @param angularVel Angular velocity (radians per second)
 * @return WheelSpeeds Left and right wheel speeds (inches/sec)
 */
WheelSpeeds diffDriveKinematics(double linearVel, double angularVel) {
    return {
        linearVel - (angularVel * TRACK_WIDTH / 2.0), // Left speed
        linearVel + (angularVel * TRACK_WIDTH / 2.0)  // Right speed
    };
}

/**
 * @brief Converts wheel speed (in inches/sec) to motor RPM
 * @param wheelSpeed Inches per second
 * @return double Motor RPM
 */
double inchesToRPM(double wheelSpeed) {
    return (wheelSpeed / WHEEL_CIRCUMFERENCE) * 60.0;
}







/////////////////      TRACJECTORY SYSTEM WPI     /////////////////////////////////////
#include <vector>

struct TrajectoryPoint {
    Pose pose;
    double velocity; // inches/sec
    double angularVelocity; // radians/sec
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;
    int size() { return points.size(); }
};

/**
 * @brief Generate a simple straight-line trajectory
 * @return Trajectory
 */
Trajectory generateTrajectory() {
    Trajectory traj;
    for (double i = 0; i <= 48; i += 2) { // Generate points every 2 inches
        traj.points.push_back({{i, 0.0, 0.0}, 40.0, 0.0});
    }
    return traj;
}




////////////////////// UPDATED ODOM ///////////////////////
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "robotConfigs.h"

Pose getOdometry() {
    // Get encoder positions in degrees for all motors
    double leftEnc = left_motors.get_position();  // Average left side
    double rightEnc = right_motors.get_position(); // Average right side

    // Convert to inches traveled
    double avgEnc = (leftEnc + rightEnc) / 2.0; // Average encoder counts
    double traveledInches = (avgEnc / 360.0) * WHEEL_CIRCUMFERENCE; // Convert to inches

    // Get heading in radians (IMU gives degrees, so convert)
    double theta = imu.get_heading() * M_PI / 180.0;

    return {traveledInches, 0.0, theta};  // Only tracking x movement for now
}



#include "pros/rtos.hpp"
void runRamseteController(Trajectory &traj) {
    int index = 0;
    while (index < traj.size()) {
        Pose currentPose = getOdometry(); // Get odometry from all motors
        TrajectoryPoint desiredPoint = traj.points[index];

        // Run RAMSETE Controller
        Velocity controlOutput = ramseteController(currentPose, desiredPoint.pose,
                                                   desiredPoint.velocity, desiredPoint.angularVelocity);

        // Convert to left/right wheel speeds
        WheelSpeeds wheelSpeeds = diffDriveKinematics(controlOutput.linear, controlOutput.angular);
        double leftRPM = inchesToRPM(wheelSpeeds.left);
        double rightRPM = inchesToRPM(wheelSpeeds.right);

        // Limit speeds
        leftRPM = std::clamp(leftRPM, -MAX_RPM, MAX_RPM);
        rightRPM = std::clamp(rightRPM, -MAX_RPM, MAX_RPM);

        // Send velocity commands to ALL motors
        left_motors.move_velocity(leftRPM);
        right_motors.move_velocity(rightRPM);

        // Move to next trajectory point
        index++;

        pros::delay(20); // Run at 50 Hz
    }
}
