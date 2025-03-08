#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include <cmath>
#include <iostream>
<<<<<<< HEAD
#include <cmath>
#include <iostream>




// ------------------------------------------------
// Global filtering variables (one per sensor)
static double frontFiltered = 0.0;
static double rightFiltered = 0.0;
static double backFiltered  = 0.0;
static double leftFiltered  = 0.0;
=======

// ------------------------------------------------
// Global filtering variables (one per sensor)
static double frontLeftFiltered = 0.0;
static double frontRightFiltered = 0.0;
static double rightFiltered = 0.0;
static double leftFiltered = 0.0;
>>>>>>> 155fe35 (first commit)

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

<<<<<<< HEAD
// ----------------------------------------------
// moveFR uses frontDist + rightDist
=======
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
>>>>>>> 155fe35 (first commit)
void moveFR(double fTarget, double rTarget,
            bool forwards,
            bool frontDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading,
            double desiredHeading)
{
<<<<<<< HEAD
    // PD Gains for each axis  double kP = 0.55, kD = 1.2;
=======
    // PD Gains for each axis
>>>>>>> 155fe35 (first commit)
    double kP_f = 0.6, kD_f = 1.81;
    double kP_r = 0.55, kD_r = 1.2;

    double prevErrorF = 0.0, prevErrorR = 0.0;
    double prevHdgError = 0.0; // for heading PD

    int startTime = pros::millis();

    // Initialize filter states
<<<<<<< HEAD
    frontFiltered = frontDist.get();
=======
    frontRightFiltered = frontDistRight.get();
>>>>>>> 155fe35 (first commit)
    rightFiltered = rightDist.get();

    while (true) {
        // Exponential smoothing
<<<<<<< HEAD
        double rawF = frontDist.get();
        double rawR = rightDist.get();
        frontFiltered = alpha * rawF + (1 - alpha) * frontFiltered;
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        // Distance error
        double errorF = frontDecreasing ? (frontFiltered - fTarget)
                                        : (fTarget - frontFiltered);
=======
        double rawF = frontDistRight.get();
        double rawR = rightDist.get();
        frontRightFiltered = alpha * rawF + (1 - alpha) * frontRightFiltered;
        rightFiltered = alpha * rawR + (1 - alpha) * rightFiltered;

        // Distance error
        double errorF = frontDecreasing ? (frontRightFiltered - fTarget)
                                        : (fTarget - frontRightFiltered);
>>>>>>> 155fe35 (first commit)
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
<<<<<<< HEAD
// moveFL uses frontDist + leftDist
=======
// moveFL uses frontDistLeft + leftDist
>>>>>>> 155fe35 (first commit)
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

<<<<<<< HEAD
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

=======
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
>>>>>>> 155fe35 (first commit)
        double dF = errorF - prevErrorF;
        double dL = errorL - prevErrorL;
        prevErrorF = errorF;
        prevErrorL = errorL;

        double powerF = (kP_f * errorF) + (kD_f * dF);
        double powerL = (kP_l * errorL) + (kD_l * dL);

<<<<<<< HEAD
=======
        // Optional heading hold
>>>>>>> 155fe35 (first commit)
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

        double turnWeight = 0.3;
<<<<<<< HEAD
        double leftPower  = powerF + (turnWeight * powerL) + turnPower;
        double rightPower = powerF - (turnWeight * powerL) - turnPower;
=======
        double leftPower  = powerF - (turnWeight * powerL) + turnPower;
        double rightPower = powerF + (turnWeight * powerL) - turnPower;
>>>>>>> 155fe35 (first commit)

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
<<<<<<< HEAD
=======

>>>>>>> 155fe35 (first commit)
    left_motors.move(0);
    right_motors.move(0);
}

// ----------------------------------------------
<<<<<<< HEAD
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
=======
// moveF uses average of both front distance sensors
>>>>>>> 155fe35 (first commit)
void moveF(double fTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
<<<<<<< HEAD
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
=======
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

>>>>>>> 155fe35 (first commit)
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

<<<<<<< HEAD
        // Tolerance
=======
>>>>>>> 155fe35 (first commit)
        if (std::fabs(error) < DIST_TOLERANCE) {
            break;
        }

        if (pros::millis() - startTime > timeOutMs) {
            std::cout << "moveF() TIMEOUT\n";
            break;
        }
        pros::delay(20);
    }
<<<<<<< HEAD
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
=======

>>>>>>> 155fe35 (first commit)
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
<<<<<<< HEAD
    double kP_r = 0.55, kD_r = 0.9;
=======
    // PD Gains
    double kP_r = 0.55, kD_r = 1.2;
>>>>>>> 155fe35 (first commit)
    double kP_l = 0.55, kD_l = 0.9;

    double prevErrorR = 0.0, prevErrorL = 0.0;
    double prevHdgError = 0.0;
    int startTime = pros::millis();

<<<<<<< HEAD
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

=======
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
>>>>>>> 155fe35 (first commit)
        double dR = errorR - prevErrorR;
        double dL = errorL - prevErrorL;
        prevErrorR = errorR;
        prevErrorL = errorL;

        double powerR = (kP_r * errorR) + (kD_r * dR);
        double powerL = (kP_l * errorL) + (kD_l * dL);

<<<<<<< HEAD
=======
        // Optional heading hold
>>>>>>> 155fe35 (first commit)
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

<<<<<<< HEAD
        double turnWeight = 0.3;
        double leftPower  = powerR + (turnWeight * powerL) + turnPower;
        double rightPower = powerR - (turnWeight * powerL) - turnPower;
=======
        // Combine powers for strafe movement
        double forwardPower = 0.0; // No forward component in RL movement
        double strafePower = (powerR + powerL) / 2.0;

        double leftPower  = forwardPower + strafePower + turnPower;
        double rightPower = forwardPower - strafePower - turnPower;
>>>>>>> 155fe35 (first commit)

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

<<<<<<< HEAD

=======
// ----------------------------------------------
// moveR uses rightDist
>>>>>>> 155fe35 (first commit)
void moveR(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
<<<<<<< HEAD
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

=======
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
>>>>>>> 155fe35 (first commit)
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

<<<<<<< HEAD
        double leftPower  = power + turnPower;
        double rightPower = power - turnPower;
=======
        // For right movement, we need to strafe
        double leftPower  = power + turnPower;
        double rightPower = -power - turnPower;
>>>>>>> 155fe35 (first commit)

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
<<<<<<< HEAD
            std::cout << "moveB() TIMEOUT\n";
=======
            std::cout << "moveR() TIMEOUT\n";
>>>>>>> 155fe35 (first commit)
            break;
        }
        pros::delay(20);
    }
<<<<<<< HEAD
=======

>>>>>>> 155fe35 (first commit)
    left_motors.move(0);
    right_motors.move(0);
}

<<<<<<< HEAD
void moveL(double rTarget,
=======
// ----------------------------------------------
// moveL uses leftDist
void moveL(double lTarget,
>>>>>>> 155fe35 (first commit)
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading,
           double desiredHeading)
{
<<<<<<< HEAD
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

=======
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
>>>>>>> 155fe35 (first commit)
        double turnPower = 0.0;
        if (holdHeading) {
            turnPower = computeHeadingPower(desiredHeading, prevHdgError);
        }

<<<<<<< HEAD
        double leftPower  = power + turnPower;
=======
        // For left movement, we need to strafe
        double leftPower  = -power + turnPower;
>>>>>>> 155fe35 (first commit)
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
<<<<<<< HEAD
            std::cout << "moveB() TIMEOUT\n";
=======
            std::cout << "moveL() TIMEOUT\n";
>>>>>>> 155fe35 (first commit)
            break;
        }
        pros::delay(20);
    }
<<<<<<< HEAD
=======

>>>>>>> 155fe35 (first commit)
    left_motors.move(0);
    right_motors.move(0);
}
