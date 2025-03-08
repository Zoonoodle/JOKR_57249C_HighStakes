#pragma once

/**
 * @file DistanceSensorMovements.hpp
 * @brief Distance-based movement functions with optional IMU heading hold and filtering.
 *
 * Each function drives the robot based on one or two distance sensors,
 * optionally maintaining a desired heading from the IMU. They use:
 *   - Exponential filtering to smooth sensor noise
 *   - A tolerance (±10 mm) around the target
 *   - PD gains for each axis, plus optional heading PD
 *   - A blocking loop that returns only when done or timed out
 */

#include <cmath>

/**
 * @brief Move using both front distance sensors (left and right) for alignment
 * 
 * @param fLeftTarget Target distance for front left sensor (mm)
 * @param fRightTarget Target distance for front right sensor (mm)
 * @param forwards True if moving forward, false if moving backward
 * @param leftDecreasing True if left distance should decrease, false if increase
 * @param rightDecreasing True if right distance should decrease, false if increase
 * @param maxSpeed Maximum motor speed (0-127)
 * @param timeOutMs Timeout in milliseconds
 * @param holdHeading Whether to maintain heading using IMU
 * @param desiredHeading Desired heading in degrees (0-360)
 */
void moveDualFront(double fLeftTarget, double fRightTarget,
                  bool forwards,
                  bool leftDecreasing, bool rightDecreasing,
                  int maxSpeed,
                  int timeOutMs,
                  bool holdHeading = false,
                  double desiredHeading = 0.0);

/**
 * @brief Move using front right and right distance sensors
 */
void moveFR(double fTarget, double rTarget,
            bool forwards,
            bool frontDecreasing, bool rightDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);

/**
 * @brief Move using front left and left distance sensors
 */
void moveFL(double fTarget, double lTarget,
            bool forwards,
            bool frontDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);

/**
 * @brief Move using front distance sensors (average of left and right)
 */
void moveF(double fTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);

/**
 * @brief Move using right and left distance sensors
 */
void moveRL(double rTarget, double lTarget,
            bool forwards,
            bool rightDecreasing, bool leftDecreasing,
            int maxSpeed,
            int timeOutMs,
            bool holdHeading = false,
            double desiredHeading = 0.0);

/**
 * @brief Move using right distance sensor
 */
void moveR(double rTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);

/**
 * @brief Move using left distance sensor
 */
void moveL(double lTarget,
           bool forwards,
           bool decreasing,
           int maxSpeed,
           int timeOutMs,
           bool holdHeading = false,
           double desiredHeading = 0.0);