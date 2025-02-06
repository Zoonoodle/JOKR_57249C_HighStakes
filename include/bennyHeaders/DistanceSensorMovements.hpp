#pragma once

/**
 * @file DistanceSensorMovements.hpp
 * @brief Distance-based movement functions with optional IMU heading hold and filtering.
 *
 * Each function drives the robot based on one or two distance sensors,
 * optionally maintaining a desired heading from the IMU. They use:
 *   - Exponential filtering to smooth sensor noise
 *   - A tolerance (±20 mm) around the target
 *   - PD gains for each axis, plus optional heading PD
 *   - A blocking loop that returns only when done or timed out
 */

#include <cmath>

void moveFR(double fTarget, double rTarget,
            bool forwards,
            bool frontDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);


void moveFL(double fTarget, double lTarget,
            bool forwards,
            bool frontDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);

void moveBR(double bTarget, double rTarget,
            bool forwards,
            bool backDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);


void moveBL(double bTarget, double lTarget,
            bool forwards,
            bool backDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);


void moveF(double fTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);


void moveB(double bTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);

void moveRL(double rTarget, double lTarget,
            bool forwards,
            bool rightDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);


void moveR(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);


void moveL(double lTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);