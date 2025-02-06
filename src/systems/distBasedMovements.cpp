#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include <cmath>
#include <iostream>
#include <cmath>
#include <iostream>




// ------------------------------------------------
// Global filtering variables (one per sensor)
static double frontFiltered = 0.0;
static double rightFiltered = 0.0;
static double backFiltered  = 0.0;
static double leftFiltered  = 0.0;

// Exponential smoothing factor
static constexpr double alpha = 1;

// PD Gains for heading hold
static constexpr double kP_h = 1.0;
static constexpr double kD_h = 2.0;

// Tolerance for distance error (mm)
static constexpr double DIST_TOLERANCE = 5.0;

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

// ----------------------------------------------
// moveFR uses frontDist + rightDist
void moveFR(double fTarget, double rTarget,
            bool forwards,
            bool frontDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    // PD Gains for each axis  double kP = 0.55, kD = 1.2;
    double kP_f = 0.6, kD_f = 1.81;
    double kP_r = 0.55, kD_r = 1.2;

    double prevErrorF = 0.0, prevErrorR = 0.0;
    double prevHdgError = 0.0; // for heading PD

    int startTime = pros::millis();

    // Initialize filter states
    frontFiltered = frontDist.get();
    rightFiltered = rightDist.get();

    while (true) {
        // Exponential smoothing
        double rawF = frontDist.get();
        double rawR = rightDist.get();
        frontFiltered = alpha * rawF + (1 - alpha) * frontFiltered;
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        // Distance error
        double errorF = frontDecreasing ? (frontFiltered - fTarget)
                                        : (fTarget - frontFiltered);
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
// moveFL uses frontDist + leftDist
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

    // Initialize filters
    frontFiltered = frontDist.get();
    leftFiltered  = leftDist.get();

    while (true) {
        double rawF = frontDist.get();
        double rawL = leftDist.get();
        frontFiltered = alpha * rawF + (1 - alpha) * frontFiltered;
        leftFiltered  = alpha * rawL + (1 - alpha) * leftFiltered;

        double errorF = frontDecreasing ? (frontFiltered - fTarget)
                                        : (fTarget - frontFiltered);
        double errorL = leftDecreasing  ? (leftFiltered - lTarget)
                                        : (lTarget - leftFiltered);

        double dF = errorF - prevErrorF;
        double dL = errorL - prevErrorL;
        prevErrorF = errorF;
        prevErrorL = errorL;

        double powerF = (kP_f * errorF) + (kD_f * dF);
        double powerL = (kP_l * errorL) + (kD_l * dL);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
        double leftPower  = powerF + (turnWeight * powerL) + turnPower;
        double rightPower = powerF - (turnWeight * powerL) - turnPower;

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
// moveBR uses backDist + rightDist
void moveBR(double bTarget, double rTarget,
            bool forwards,
            bool backDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    double kP_b = 0.6, kD_b = 1.81;
    double kP_r = 0.55, kD_r = 0.9;

    double prevErrorB = 0.0, prevErrorR = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    // initialize filters
    backFiltered  = backDist.get();
    rightFiltered = rightDist.get();

    while (true) {
        double rawB = backDist.get();
        double rawR = rightDist.get();
        backFiltered  = alpha * rawB + (1 - alpha) * backFiltered;
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        double errorB = backDecreasing  ? (backFiltered - bTarget)
                                        : (bTarget - backFiltered);
        double errorR = rightDecreasing ? (rightFiltered - rTarget)
                                        : (rTarget - rightFiltered);

        double dB = errorB - prevErrorB;
        double dR = errorR - prevErrorR;
        prevErrorB = errorB;
        prevErrorR = errorR;

        double powerB = (kP_b * errorB) + (kD_b * dB);
        double powerR = (kP_r * errorR) + (kD_r * dR);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
        double leftPower  = powerB + (turnWeight * powerR) + turnPower;
        double rightPower = powerB - (turnWeight * powerR) - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool bDone = (std::fabs(errorB) < DIST_TOLERANCE);
        bool rDone = (std::fabs(errorR) < DIST_TOLERANCE);
        if (bDone && rDone) break;

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveBR() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveBL uses backDist + leftDist
void moveBL(double bTarget, double lTarget,
            bool forwards,
            bool backDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
    double kP_b = 0.6, kD_b = 1.81;
    double kP_l = 0.55, kD_l = 0.9;

    double prevErrorB = 0.0, prevErrorL = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    backFiltered = backDist.get();
    leftFiltered = leftDist.get();

    while (true) {
        double rawB = backDist.get();
        double rawL = leftDist.get();
        backFiltered = alpha * rawB + (1 - alpha) * backFiltered;
        leftFiltered = alpha * rawL + (1 - alpha) * leftFiltered;

        double errorB = backDecreasing ? (backFiltered - bTarget)
                                       : (bTarget - backFiltered);
        double errorL = leftDecreasing ? (leftFiltered - lTarget)
                                       : (lTarget - leftFiltered);

        double dB = errorB - prevErrorB;
        double dL = errorL - prevErrorL;
        prevErrorB = errorB;
        prevErrorL = errorL;

        double powerB = (kP_b * errorB) + (kD_b * dB);
        double powerL = (kP_l * errorL) + (kD_l * dL);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
        double leftPower  = powerB + (turnWeight * powerL) + turnPower;
        double rightPower = powerB - (turnWeight * powerL) - turnPower;

        if (!forwards) {
            leftPower  = -leftPower;
            rightPower = -rightPower;
        }

        leftPower  = clamp(leftPower, -maxSpeed, maxSpeed);
        rightPower = clamp(rightPower, -maxSpeed, maxSpeed);

        left_motors.move(leftPower);
        right_motors.move(rightPower);

        bool bDone = (std::fabs(errorB) < DIST_TOLERANCE);
        bool lDone = (std::fabs(errorL) < DIST_TOLERANCE);
        if (bDone && lDone) break;

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveBL() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
// moveF uses only frontDist
void moveF(double fTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
      double kP = 0.6, kD = 1.81;
    double prevError = 0.0;
    double prevHdgError = 0.0;

    int startTime = pros::millis();

    // init filter
    frontFiltered = frontDist.get();

    while (true) {
        double rawF = frontDist.get();
        frontFiltered = alpha * rawF + (1 - alpha) * frontFiltered;

        double error = decreasing ? (frontFiltered - fTarget)
                                  : (fTarget - frontFiltered);

        double dE = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * dE);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        // For single-sensor forward motion, we might just do:
        // leftPower = power + turnPower
        // rightPower= power - turnPower
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

        // Tolerance
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
// moveB uses only backDist
void moveB(double bTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    double kP = 0.6, kD = 1.81;
    double prevError = 0.0;
    double prevHdgError = 0.0;

    int startTime = pros::millis();

    // init filter
    backFiltered = backDist.get();

    while (true) {
        double rawB = backDist.get();
        backFiltered = alpha * rawB + (1 - alpha) * backFiltered;

        double error = decreasing ? (backFiltered - bTarget)
                                  : (bTarget - backFiltered);

        double dE = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * dE);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
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
            std::cout << "moveB() TIMEOUT\n";
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
    double kP_r = 0.55, kD_r = 0.9;
    double kP_l = 0.55, kD_l = 0.9;

    double prevErrorR = 0.0, prevErrorL = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

    rightFiltered = rightDist.get();
    leftFiltered  = leftDist.get();

    while (true) {
        double rawR = rightDist.get();
        double rawL = leftDist.get();
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;
        leftFiltered  = alpha * rawL + (1 - alpha) * leftFiltered;

        double errorR = rightDecreasing ? (rightFiltered - rTarget)
                                        : (rTarget - rightFiltered);
        double errorL = leftDecreasing  ? (leftFiltered - lTarget)
                                        : (lTarget - leftFiltered);

        double dR = errorR - prevErrorR;
        double dL = errorL - prevErrorL;
        prevErrorR = errorR;
        prevErrorL = errorL;

        double powerR = (kP_r * errorR) + (kD_r * dR);
        double powerL = (kP_l * errorL) + (kD_l * dL);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
        double leftPower  = powerR + (turnWeight * powerL) + turnPower;
        double rightPower = powerR - (turnWeight * powerL) - turnPower;

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


void moveR(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    double kP = 0.52, kD = 1.2;
    double prevError = 0.0;
    double prevHdgError = 0.0;

    int startTime = pros::millis();

    // init filter
    backFiltered = rightDist.get();

    while (true) {
        double rawB = rightDist.get();
        rightFiltered = alpha * rawB + (1 - alpha) * rightFiltered;

        double error = decreasing ? (rightFiltered - rTarget)
                                  : (rTarget - rightFiltered);

        double dE = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * dE);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
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
            std::cout << "moveB() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}

void moveL(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
    double kP = 0.52, kD = 1.2;
    double prevError = 0.0;
    double prevHdgError = 0.0;

    int startTime = pros::millis();

    // init filter
    backFiltered = leftDist.get();

    while (true) {
        double rawB = leftDist.get();
        rightFiltered = alpha * rawB + (1 - alpha) * rightFiltered;

        double error = decreasing ? (rightFiltered - rTarget)
                                  : (rTarget - rightFiltered);

        double dE = error - prevError;
        prevError = error;

        double power = (kP * error) + (kD * dE);

        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
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
            std::cout << "moveB() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
    left_motors.move(0);
    right_motors.move(0);
}
