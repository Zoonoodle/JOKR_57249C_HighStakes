#pragma once

#include "lemlib/chassis/chassis.hpp"
#include <vector>

/**
 * @brief Initialize the Monte Carlo Localization system
 * 
 * @param initialX Initial X position in mm
 * @param initialY Initial Y position in mm
 * @param initialTheta Initial orientation in degrees (0-360)
 * @param useStandardField Whether to set up standard VEX field walls
 * @param particleCount Number of particles to use (default: 500)
 */
void initializeMCL(double initialX, double initialY, double initialTheta, 
                  bool useStandardField = true, int particleCount = 500);

/**
 * @brief Add a custom wall to the field simulation
 * 
 * @param x1 Start X coordinate (mm)
 * @param y1 Start Y coordinate (mm)
 * @param x2 End X coordinate (mm)
 * @param y2 End Y coordinate (mm)
 */
void addMCLWall(double x1, double y1, double x2, double y2);

/**
 * @brief Get the current best estimate of the robot's position
 * 
 * @return lemlib::Pose The estimated pose (x, y, theta)
 */
lemlib::Pose getMCLPose();

/**
 * @brief Override the current MCL position estimate
 * 
 * Use this when you have definitive knowledge of the robot's position
 * (e.g., at the start of autonomous)
 * 
 * @param x X position in mm
 * @param y Y position in mm
 * @param theta Orientation in degrees
 */
void setMCLPosition(double x, double y, double theta);

/**
 * @brief Reset the particle filter to current odometry with uncertainty
 */
void resetMCL();

/**
 * @brief Stop the MCL system
 */
void stopMCL();

/**
 * @brief Display the MCL visualization on the PROS screen
 */
void displayMCL();

/**
 * @brief Handle screen touch events for the MCL visualization
 */
void handleMCLTouch();

/**
 * @brief Simulate an autonomous path with the MCL system
 * 
 * @param waypoints Vector of waypoints (x, y, theta) defining the path
 */
void simulateMCLPath(const std::vector<lemlib::Pose>& waypoints);

/**
 * @brief Create and simulate a path from the current position to a target position
 * 
 * @param targetX Target X position in mm
 * @param targetY Target Y position in mm
 * @param targetTheta Target orientation in degrees
 * @param numWaypoints Number of waypoints to generate along the path
 */
void simulatePathToTarget(double targetX, double targetY, double targetTheta, int numWaypoints = 5);

/**
 * @brief Function to be called from autonomous init to set up MCL
 * 
 * @param initialX Initial X position in mm
 * @param initialY Initial Y position in mm
 * @param initialTheta Initial orientation in degrees
 */
void setupMCLForAutonomous(double initialX, double initialY, double initialTheta);

/**
 * @brief Get a fused position estimate from MCL and odometry
 * 
 * @param mclWeight Weight to give to MCL estimate (0.0 to 1.0)
 * @return lemlib::Pose Fused position estimate
 */
lemlib::Pose getFusedPosition(double mclWeight = 0.7);

/**
 * @brief Adjust the robot's position using the MCL estimate
 * 
 * This is useful for correcting odometry drift using MCL
 */
void correctOdometryWithMCL(); 