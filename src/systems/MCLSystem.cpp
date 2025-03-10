#include "bennyHeaders/MonteCarloLocalization.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "display/lvgl.h"
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

// Forward declaration of the MCL update loop function
void mclUpdateLoop(void*);

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
                  bool useStandardField = true, int particleCount = 500) {
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
            particleCount        // Number of particles
        );
        
        // Create visualizer
        mclVisualizer = std::make_unique<MCLVisualizer>(*mcl);
    }
    
    // Initialize particles around the initial position
    mcl->initialize(
        initialX,
        initialY,
        initialTheta * DEG_TO_RAD, // Convert degrees to radians
        50.0,  // Initial X spread
        50.0,  // Initial Y spread
        0.1    // Initial angle spread (radians)
    );
    
    // Set up the field walls if requested
    if (useStandardField) {
        mcl->setupStandardField();
    }
    
    // Set appropriate noise model parameters based on IMU presence
    // Lower rotation noise since IMU provides better rotational accuracy
    mcl->setMotionModelParams(
        0.1,  // Alpha1: Translation noise from translation (higher = more noise)
        0.05, // Alpha2: Translation noise from rotation (reduced with IMU)
        0.05, // Alpha3: Rotation noise from translation (reduced with IMU)
        0.05  // Alpha4: Rotation noise from rotation (reduced with IMU)
    );
    
    // Set sensor model parameters
    mcl->setSensorModelParams(
        50.0,              // Standard deviation of sensor noise (mm)
        MAX_SENSOR_DISTANCE // Maximum reliable sensor distance (mm)
    );
    
    // Create the update task if not already running
    if (!mclRunning) {
        mclRunning = true;
        mclTask = new pros::Task(mclUpdateLoop);
    }
    
    // Display the visualizer
    mclVisualizer->display();
}

/**
 * @brief Add a custom wall to the field simulation
 * 
 * @param x1 Start X coordinate (mm)
 * @param y1 Start Y coordinate (mm)
 * @param x2 End X coordinate (mm)
 * @param y2 End Y coordinate (mm)
 */
void addMCLWall(double x1, double y1, double x2, double y2) {
    if (mcl) {
        mcl->addWall(x1, y1, x2, y2);
    }
}

/**
 * @brief Get the current best estimate of the robot's position
 * 
 * @return lemlib::Pose The estimated pose (x, y, theta)
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
 * Use this when you have definitive knowledge of the robot's position
 * (e.g., at the start of autonomous)
 * 
 * @param x X position in mm
 * @param y Y position in mm
 * @param theta Orientation in degrees
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
 * @param waypoints Vector of waypoints (x, y, theta) defining the path
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
 * @param targetX Target X position in mm
 * @param targetY Target Y position in mm
 * @param targetTheta Target orientation in degrees
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
 * @param initialX Initial X position in mm
 * @param initialY Initial Y position in mm
 * @param initialTheta Initial orientation in degrees
 */
void setupMCLForAutonomous(double initialX, double initialY, double initialTheta) {
    // Initialize MCL system with standard field
    initializeMCL(initialX, initialY, initialTheta);
    
    // Update the odometry position to match the MCL position
    chassis.setPose(initialX, initialY, initialTheta);
}

/**
 * @brief Get a fused position estimate from MCL and odometry
 * 
 * @param mclWeight Weight to give to MCL estimate (0.0 to 1.0)
 * @return lemlib::Pose Fused position estimate
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