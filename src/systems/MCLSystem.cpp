#include "bennyHeaders/MonteCarloLocalization.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "pros/apix.h"  // Correct LVGL include for PROS
#include <memory>
#include <vector>
#include <functional>

// Global MCL instance
static std::unique_ptr<MonteCarloLocalization> mcl;
static std::unique_ptr<MCLVisualizer> mclVisualizer;

// Flag to indicate if the MCL system is running
static bool mclRunning = false;

// Task handle for the MCL update loop
static pros::Task* mclTask = nullptr;

// Maximum distance for sensors (in mm)
static constexpr double MAX_SENSOR_DISTANCE = 2000.0;

// The update rate for the MCL system (in milliseconds)
static constexpr int MCL_UPDATE_INTERVAL_MS = 50;

// Store initial sensor readings for relative measurement
static double initialLeftReading = 0.0;
static double initialRightReading = 0.0;
static double initialFrontLeftReading = 0.0;
static double initialFrontRightReading = 0.0;

// Initial heading from IMU (for relative heading)
static double initialHeading = 0.0;

// Forward declaration of the MCL update loop function
void mclUpdateLoop(void*);

/**
 * @brief Initialize the Monte Carlo Localization system
 * 
 * Sets up the MCL system with the robot's current position as (0,0,initial_heading)
 * All subsequent measurements will be relative to this starting position
 * 
 * @param initialHeadingDeg Initial heading in degrees (0-360)
 * @param particleCount Number of particles to use (default: 500)
 */
void initializeMCL(double initialHeadingDeg = 0.0, int particleCount = 500) {
    // Store initial sensor readings for relative measurement
    initialFrontLeftReading = frontDistLeft.get();
    initialFrontRightReading = frontDistRight.get();
    initialLeftReading = leftDist.get();
    initialRightReading = rightDist.get();
    
    // Store initial heading (for relative orientation)
    initialHeading = imu.get_heading();
    
    // Create new MCL instance if not already created
    if (!mcl) {
        // Create MCL instance with references to sensors
        mcl = std::make_unique<MonteCarloLocalization>(
            chassis,             // LemLib chassis
            frontDistLeft,       // Front left distance sensor
            frontDistRight,      // Front right distance sensor
            rightDist,           // Right distance sensor
            leftDist,            // Left distance sensor
            imu,                 // IMU sensor for heading
            particleCount,       // Number of particles
            initialFrontLeftReading,   // Initial front left reading
            initialFrontRightReading,  // Initial front right reading 
            initialLeftReading,        // Initial left reading
            initialRightReading,       // Initial right reading
            initialHeading             // Initial heading
        );
        
        // Create visualizer
        mclVisualizer = std::make_unique<MCLVisualizer>(*mcl);
    }
    
    // Initialize particles at (0,0,initialHeadingDeg)
    mcl->initialize(
        0.0,  // X = 0
        0.0,  // Y = 0
        initialHeadingDeg * DEG_TO_RAD, // Initial heading in radians
        10.0,  // Initial X spread
        10.0,  // Initial Y spread
        0.05   // Initial angle spread (radians)
    );
    
    // Set relative walls based on initial distance readings
    // We're making walls at the estimated positions relative to our (0,0)
    // Note: These are approximate and assume walls are perpendicular
    
    // Right wall (if detected)
    if (initialRightReading < MAX_SENSOR_DISTANCE) {
        mcl->addWall(0, -initialRightReading, 2000, -initialRightReading);
    }
    
    // Left wall (if detected)
    if (initialLeftReading < MAX_SENSOR_DISTANCE) {
        mcl->addWall(0, initialLeftReading, 2000, initialLeftReading);
    }
    
    // Front wall (if detected)
    if (initialFrontLeftReading < MAX_SENSOR_DISTANCE || 
        initialFrontRightReading < MAX_SENSOR_DISTANCE) {
        // Use the average of both readings if available
        double avgFrontDist = (initialFrontLeftReading < MAX_SENSOR_DISTANCE && 
                              initialFrontRightReading < MAX_SENSOR_DISTANCE) ?
                              (initialFrontLeftReading + initialFrontRightReading) / 2.0 :
                              (initialFrontLeftReading < MAX_SENSOR_DISTANCE ? 
                               initialFrontLeftReading : initialFrontRightReading);
                               
        mcl->addWall(-initialLeftReading, avgFrontDist, initialRightReading, avgFrontDist);
    }
    
    // Set appropriate noise model parameters
    mcl->setMotionModelParams(
        0.1,  // Alpha1: Translation noise from translation
        0.05, // Alpha2: Translation noise from rotation
        0.05, // Alpha3: Rotation noise from translation
        0.05  // Alpha4: Rotation noise from rotation
    );
    
    // Set sensor model parameters
    mcl->setSensorModelParams(
        50.0,              // Standard deviation of sensor noise (mm)
        MAX_SENSOR_DISTANCE // Maximum reliable sensor distance (mm)
    );
    
    // Reset odometry to match our relative coordinate system
    chassis.setPose(0, 0, initialHeadingDeg);
    
    // Create the update task if not already running
    if (!mclRunning) {
        mclRunning = true;
        mclTask = new pros::Task(mclUpdateLoop);
    }
    
    // Display the visualizer
    mclVisualizer->display();
}

/**
 * @brief Add a custom wall to the relative coordinate system
 * 
 * Wall coordinates are relative to the robot's starting position
 * 
 * @param x1 Start X coordinate relative to start position (mm)
 * @param y1 Start Y coordinate relative to start position (mm)
 * @param x2 End X coordinate relative to start position (mm)
 * @param y2 End Y coordinate relative to start position (mm)
 */
void addMCLWall(double x1, double y1, double x2, double y2) {
    if (mcl) {
        mcl->addWall(x1, y1, x2, y2);
    }
}

/**
 * @brief Get the current position estimate relative to starting position
 * 
 * @return lemlib::Pose The estimated pose (x, y, theta) relative to start
 */
lemlib::Pose getMCLPose() {
    if (mcl) {
        return mcl->getEstimatedPose();
    }
    // If MCL is not initialized, return the odometry pose
    return chassis.getPose();
}

/**
 * @brief Override the current MCL position estimate
 * 
 * Sets the robot's current position in the relative coordinate system
 * 
 * @param x X position relative to start (mm)
 * @param y Y position relative to start (mm)
 * @param theta Orientation in degrees relative to initial heading
 */
void setMCLPosition(double x, double y, double theta) {
    if (mcl) {
        mcl->setPosition(x, y, theta * DEG_TO_RAD);
    }
}

/**
 * @brief Reset the particle filter to current odometry with uncertainty
 */
void resetMCL() {
    if (mcl) {
        mcl->reset();
    }
}

/**
 * @brief Stop the MCL system
 */
void stopMCL() {
    mclRunning = false;
    if (mclTask) {
        mclTask->remove();
        delete mclTask;
        mclTask = nullptr;
    }
}

/**
 * @brief Display the MCL visualization
 */
void displayMCL() {
    if (mclVisualizer) {
        mclVisualizer->display();
    }
}

/**
 * @brief Handle screen touch events for the MCL visualization
 */
void handleMCLTouch() {
    if (mclVisualizer) {
        mclVisualizer->handleTouch();
    }
}

/**
 * @brief Simulate an autonomous path with the MCL system
 * 
 * Path waypoints are relative to the robot's starting position
 * 
 * @param waypoints Vector of waypoints (x, y, theta) relative to start position
 */
void simulateMCLPath(const std::vector<lemlib::Pose>& waypoints) {
    if (mcl && !waypoints.empty()) {
        // Stop the update task temporarily
        bool wasRunning = mclRunning;
        if (wasRunning) {
            stopMCL();
        }
        
        // Run the simulation
        mcl->simulateAutonomousPath(waypoints, [](const lemlib::Pose& pose, const std::vector<MonteCarloLocalization::Particle>& particles) {
            // Display the visualization during simulation
            if (mclVisualizer) {
                mclVisualizer->display();
            }
        });
        
        // Restart the update task if it was running before
        if (wasRunning) {
            mclRunning = true;
            mclTask = new pros::Task(mclUpdateLoop);
        }
    }
}

/**
 * @brief MCL update loop, runs as a task
 */
void mclUpdateLoop(void*) {
    while (mclRunning) {
        if (mcl) {
            // Update the particle filter
            mcl->update();
            
            // Display the visualization periodically (less frequently than updates)
            static int visualCounter = 0;
            if (++visualCounter >= 5) { // Visualize every 5 updates
                visualCounter = 0;
                displayMCL();
            }
        }
        pros::delay(MCL_UPDATE_INTERVAL_MS);
    }
}

/**
 * @brief Create and simulate a path from the current position to a target position
 * 
 * @param targetX Target X position relative to start (mm)
 * @param targetY Target Y position relative to start (mm)
 * @param targetTheta Target orientation in degrees relative to initial heading
 * @param numWaypoints Number of waypoints to generate along the path
 */
void simulatePathToTarget(double targetX, double targetY, double targetTheta, int numWaypoints = 5) {
    if (!mcl) return;
    
    lemlib::Pose currentPose = mcl->getEstimatedPose();
    std::vector<lemlib::Pose> path;
    
    // Add current position as first waypoint
    path.push_back(currentPose);
    
    // Generate intermediate waypoints
    for (int i = 1; i < numWaypoints; i++) {
        double t = static_cast<double>(i) / numWaypoints;
        double x = currentPose.x + t * (targetX - currentPose.x);
        double y = currentPose.y + t * (targetY - currentPose.y);
        double theta = currentPose.theta + t * (targetTheta - currentPose.theta);
        
        // Normalize theta to [0, 360)
        while (theta >= 360.0) theta -= 360.0;
        while (theta < 0.0) theta += 360.0;
        
        path.push_back(lemlib::Pose(x, y, theta));
    }
    
    // Add target position as last waypoint
    path.push_back(lemlib::Pose(targetX, targetY, targetTheta));
    
    // Run the simulation
    simulateMCLPath(path);
}

/**
 * @brief Function to be called from autonomous init to set up MCL
 * 
 * @param initialHeadingDeg Initial heading in degrees
 */
void setupMCLForAutonomous(double initialHeadingDeg = 0.0) {
    // Initialize MCL system with the current position as (0,0)
    initializeMCL(initialHeadingDeg);
    
    // Update the odometry position to match our relative coordinate system
    chassis.setPose(0, 0, initialHeadingDeg);
}

/**
 * @brief Get a fused position estimate from MCL and odometry
 * 
 * @param mclWeight Weight to give to MCL estimate (0.0 to 1.0)
 * @return lemlib::Pose Fused position estimate relative to start position
 */
lemlib::Pose getFusedPosition(double mclWeight = 0.7) {
    if (!mcl) {
        return chassis.getPose();
    }
    
    // Clamp weight to [0, 1]
    mclWeight = std::max(0.0, std::min(1.0, mclWeight));
    double odomWeight = 1.0 - mclWeight;
    
    // Get positions from both systems
    lemlib::Pose mclPose = mcl->getEstimatedPose();
    lemlib::Pose odomPose = chassis.getPose();
    
    // Calculate weighted average of positions
    double x = mclPose.x * mclWeight + odomPose.x * odomWeight;
    double y = mclPose.y * mclWeight + odomPose.y * odomWeight;
    
    // Special handling for angle to handle wraparound
    double mclTheta = mclPose.theta * DEG_TO_RAD;
    double odomTheta = odomPose.theta * DEG_TO_RAD;
    
    // Calculate angle difference and normalize to [-π, π]
    double angleDiff = mclTheta - odomTheta;
    while (angleDiff > M_PI) angleDiff -= 2.0 * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2.0 * M_PI;
    
    // Apply weighted angle difference
    double theta = odomTheta + angleDiff * mclWeight;
    
    // Normalize theta back to [0, 2π)
    while (theta < 0) theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI) theta -= 2.0 * M_PI;
    
    // Convert back to degrees
    theta *= RAD_TO_DEG;
    
    return lemlib::Pose(x, y, theta);
}

/**
 * @brief Adjust the robot's position using the MCL estimate
 * 
 * This is useful for correcting odometry drift using MCL
 */
void correctOdometryWithMCL() {
    if (mcl) {
        lemlib::Pose mclPose = mcl->getEstimatedPose();
        chassis.setPose(mclPose.x, mclPose.y, mclPose.theta);
    }
} 
