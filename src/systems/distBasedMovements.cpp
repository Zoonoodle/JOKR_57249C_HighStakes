#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include <cmath>
#include <iostream>

// ------------------------------------------------
// Global filtering variables (one per sensor)
static double frontLeftFiltered = 0.0;
static double frontRightFiltered = 0.0;
static double rightFiltered = 0.0;
static double leftFiltered = 0.0;

// Exponential smoothing factor
static constexpr double alpha = 1.1;

// PD Gains for heading hold
static constexpr double kP_h = 1.0;
static constexpr double kD_h = 2.0;

// Tolerance for distance error (mm)
static constexpr double DIST_TOLERANCE = 10.0;

inline double clamp(double value, double minVal, double maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

/**
 * @brief Helper for heading hold
 */
static double computeHeadingPower(double desiredHeading, double &prevHdgError) {
    double imuHdg   = imu.get_heading(); // 0..360
    double hdgError = desiredHeading - imuHdg;
    // wrap to [-180..180]
    if (hdgError > 180)  hdgError -= 360;
    if (hdgError < -180) hdgError += 360;

    double dHdg = hdgError - prevHdgError;
    prevHdgError = hdgError;

    double turnPower = (kP_h * hdgError) + (kD_h * dHdg);
    return turnPower;
}

/**
 * @brief Get average front distance from both front sensors
 */
static double getAverageFrontDistance() {
    return (frontDistLeft.get() + frontDistRight.get()) / 2.0;
}

// ----------------------------------------------
// moveDualFront uses both front distance sensors for alignment
void moveDualFront(double fLeftTarget, double fRightTarget,
                  bool forwards,
                  bool leftDecreasing, bool rightDecreasing,
                  int maxSpeed,
                  int timeOutMs,
                  bool holdHeading,
                  double desiredHeading)
{
    // PD Gains for each axis
    double kP_fL = 0.6, kD_fL = 1.81;
    double kP_fR = 0.6, kD_fR = 1.81;

    double prevErrorFL = 0.0, prevErrorFR = 0.0;
    double prevHdgError = 0.0; // for heading PD

    int startTime = pros::millis();

    // Initialize filter states
    frontLeftFiltered = frontDistLeft.get();
    frontRightFiltered = frontDistRight.get();

    while (true) {
        // Exponential smoothing
        double rawFL = frontDistLeft.get();
        double rawFR = frontDistRight.get();
        frontLeftFiltered = alpha * rawFL + (1 - alpha) * frontLeftFiltered;
        frontRightFiltered = alpha * rawFR + (1 - alpha) * frontRightFiltered;

        // Distance error
        double errorFL = leftDecreasing ? (frontLeftFiltered - fLeftTarget)
                                       : (fLeftTarget - frontLeftFiltered);
        double errorFR = rightDecreasing ? (frontRightFiltered - fRightTarget)
                                        : (fRightTarget - frontRightFiltered);

        // PD
        double dFL = errorFL - prevErrorFL;
        double dFR = errorFR - prevErrorFR;
        prevErrorFL = errorFL;
        prevErrorFR = errorFR;

        double powerFL = (kP_fL * errorFL) + (kD_fL * dFL);
        double powerFR = (kP_fR * errorFR) + (kD_fR * dFR);

        // Average the powers for forward movement
        double forwardPower = (powerFL + powerFR) / 2.0;
        
        // Difference for turning correction
        double turnCorrection = (powerFL - powerFR) * 0.5;

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        } else {
            // If not holding heading, use the sensor-based correction
            turnPower = turnCorrection;
        }

        double leftPower = forwardPower + turnPower;
        double rightPower = forwardPower - turnPower;

        if (!forwards) {
            leftPower = -leftPower;
            rightPower = -rightPower;
        }

        leftPower = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool flDone = (std::fabs(errorFL) < DIST_TOLERANCE);
        bool frDone = (std::fabs(errorFR) < DIST_TOLERANCE);
        if (flDone && frDone) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveDualFront() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveFR uses frontDistRight + rightDist
void moveFR(double fTarget, double rTarget,
            bool forwards,
            bool frontDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    // PD Gains for each axis
    double kP_f = 0.6, kD_f = 1.81;
    double kP_r = 0.55, kD_r = 1.2;

    double prevErrorF = 0.0, prevErrorR = 0.0;
    double prevHdgError = 0.0; // for heading PD

    int startTime = pros::millis();

    // Initialize filter states
    frontRightFiltered = frontDistRight.get();
    rightFiltered = rightDist.get();

    while (true) {
        // Exponential smoothing
        double rawF = frontDistRight.get();
        double rawR = rightDist.get();
        frontRightFiltered = alpha * rawF + (1 - alpha) * frontRightFiltered;
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        // Distance error
        double errorF = frontDecreasing ? (frontRightFiltered - fTarget)
                                        : (fTarget - frontRightFiltered);
        double errorR = rightDecreasing ? (rightFiltered - rTarget)
                                        : (rTarget - rightFiltered);

        // PD
        double dF = errorF - prevErrorF;
        double dR = errorR - prevErrorR;
        prevErrorF = errorF;
        prevErrorR = errorR;

        double powerF = (kP_f * errorF) + (kD_f * dF);
        double powerR = (kP_r * errorR) + (kD_r * dR);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3; 
        double leftPower  = powerF + (turnWeight * powerR) + turnPower;
        double rightPower = powerF - (turnWeight * powerR) - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool fDone = (std::fabs(errorF) < DIST_TOLERANCE);
        bool rDone = (std::fabs(errorR) < DIST_TOLERANCE);
        if (fDone && rDone) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveFR() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveFL uses frontDistLeft + leftDist
void moveFL(double fTarget, double lTarget,
            bool forwards,
            bool frontDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    // PD Gains
    double kP_f = 0.6, kD_f = 1.81;
    double kP_l = 0.55, kD_l = 0.9;

    double prevErrorF = 0.0, prevErrorL = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // Initialize filter states
    frontLeftFiltered = frontDistLeft.get();
    leftFiltered = leftDist.get();

    while (true) {
        // Exponential smoothing
        double rawF = frontDistLeft.get();
        double rawL = leftDist.get();
        frontLeftFiltered = alpha * rawF + (1 - alpha) * frontLeftFiltered;
        leftFiltered = alpha * rawL + (1 - alpha) * leftFiltered;

        // Distance error
        double errorF = frontDecreasing ? (frontLeftFiltered - fTarget)
                                        : (fTarget - frontLeftFiltered);
        double errorL = leftDecreasing ? (leftFiltered - lTarget)
                                       : (lTarget - leftFiltered);

        // PD
        double dF = errorF - prevErrorF;
        double dL = errorL - prevErrorL;
        prevErrorF = errorF;
        prevErrorL = errorL;

        double powerF = (kP_f * errorF) + (kD_f * dF);
        double powerL = (kP_l * errorL) + (kD_l * dL);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
        double leftPower  = powerF - (turnWeight * powerL) + turnPower;
        double rightPower = powerF + (turnWeight * powerL) - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool fDone = (std::fabs(errorF) < DIST_TOLERANCE);
        bool lDone = (std::fabs(errorL) < DIST_TOLERANCE);
        if (fDone && lDone) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveFL() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveF uses average of both front distance sensors
void moveF(double fTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    // PD Gains
    double kP = 0.6, kD = 1.81;

    double prevError = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // Initialize filter states
    frontLeftFiltered = frontDistLeft.get();
    frontRightFiltered = frontDistRight.get();

    while (true) {
        // Exponential smoothing
        double rawFL = frontDistLeft.get();
        double rawFR = frontDistRight.get();
        frontLeftFiltered = alpha * rawFL + (1 - alpha) * frontLeftFiltered;
        frontRightFiltered = alpha * rawFR + (1 - alpha) * frontRightFiltered;
        
        // Use average of both front sensors
        double frontAvg = (frontLeftFiltered + frontRightFiltered) / 2.0;

        // Distance error
        double error = decreasing ? (frontAvg - fTarget)
                                  : (fTarget - frontAvg);

        // PD
        double d = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * d);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        } else {
            // If not using heading hold, use the difference between sensors for alignment
            double sensorDiff = frontLeftFiltered - frontRightFiltered;
            turnPower = sensorDiff * 0.05; // Small correction factor
        }

        double leftPower  = power + turnPower;
        double rightPower = power - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        if (std::fabs(error) < DIST_TOLERANCE) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveF() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveRL uses rightDist + leftDist
void moveRL(double rTarget, double lTarget,
            bool forwards,
            bool rightDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    // PD Gains
    double kP_r = 0.55, kD_r = 1.2;
    double kP_l = 0.55, kD_l = 0.9;

    double prevErrorR = 0.0, prevErrorL = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // Initialize filter states
    rightFiltered = rightDist.get();
    leftFiltered = leftDist.get();

    while (true) {
        // Exponential smoothing
        double rawR = rightDist.get();
        double rawL = leftDist.get();
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;
        leftFiltered = alpha * rawL + (1 - alpha) * leftFiltered;

        // Distance error
        double errorR = rightDecreasing ? (rightFiltered - rTarget)
                                        : (rTarget - rightFiltered);
        double errorL = leftDecreasing ? (leftFiltered - lTarget)
                                       : (lTarget - leftFiltered);

        // PD
        double dR = errorR - prevErrorR;
        double dL = errorL - prevErrorL;
        prevErrorR = errorR;
        prevErrorL = errorL;

        double powerR = (kP_r * errorR) + (kD_r * dR);
        double powerL = (kP_l * errorL) + (kD_l * dL);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        // Combine powers for strafe movement
        double forwardPower = 0.0; // No forward component in RL movement
        double strafePower = (powerR + powerL) / 2.0;

        double leftPower  = forwardPower + strafePower + turnPower;
        double rightPower = forwardPower - strafePower - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool rDone = (std::fabs(errorR) < DIST_TOLERANCE);
        bool lDone = (std::fabs(errorL) < DIST_TOLERANCE);
        if (rDone && lDone) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveRL() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveR uses rightDist
void moveR(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    // PD Gains
    double kP = 0.55, kD = 1.2;

    double prevError = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // Initialize filter state
    rightFiltered = rightDist.get();

    while (true) {
        // Exponential smoothing
        double rawR = rightDist.get();
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        // Distance error
        double error = decreasing ? (rightFiltered - rTarget)
                                  : (rTarget - rightFiltered);

        // PD
        double d = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * d);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        // For right movement, we need to strafe
        double leftPower  = power + turnPower;
        double rightPower = -power - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        if (std::fabs(error) < DIST_TOLERANCE) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveR() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveL uses leftDist
void moveL(double lTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    // PD Gains
    double kP = 0.55, kD = 0.9;

    double prevError = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // Initialize filter state
    leftFiltered = leftDist.get();

    while (true) {
        // Exponential smoothing
        double rawL = leftDist.get();
        leftFiltered = alpha * rawL + (1 - alpha) * leftFiltered;

        // Distance error
        double error = decreasing ? (leftFiltered - lTarget)
                                  : (lTarget - leftFiltered);

        // PD
        double d = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * d);

        // Optional heading hold
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        // For left movement, we need to strafe
        double leftPower  = -power + turnPower;
        double rightPower = power - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        if (std::fabs(error) < DIST_TOLERANCE) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveL() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}
