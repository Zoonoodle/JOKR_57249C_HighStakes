#pragma once

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include <vector>
#include <random>
#include <memory>
#include <functional>

/**
 * @brief Monte Carlo Localization (Particle Filter) for VEX robots
 * 
 * This class implements a particle filter that fuses odometry and distance sensor data
 * to achieve more accurate localization during autonomous routines.
 * 
 * The coordinate system is relative to the robot's starting position (0,0).
 */
class MonteCarloLocalization {
public:
    /**
     * @brief Struct representing a single particle (possible robot position and orientation)
     */
    struct Particle {
        double x;           // X position in mm (relative to start)
        double y;           // Y position in mm (relative to start)
        double theta;       // Orientation in radians (relative to initial heading)
        double weight;      // Particle weight (likelihood)
        
        Particle(double x, double y, double theta, double weight = 1.0)
            : x(x), y(y), theta(theta), weight(weight) {}
    };
    
    /**
     * @brief Struct representing field walls for simulation
     */
    struct Wall {
        double x1, y1;      // Start point (relative to starting position)
        double x2, y2;      // End point (relative to starting position)
        
        Wall(double x1, double y1, double x2, double y2)
            : x1(x1), y1(y1), x2(x2), y2(y2) {}
    };

    /**
     * @brief Constructor for the Monte Carlo Localization system
     * 
     * @param chassis Reference to the LemLib chassis for odometry data
     * @param frontDistLeft Front left distance sensor
     * @param frontDistRight Front right distance sensor
     * @param rightDist Right distance sensor
     * @param leftDist Left distance sensor
     * @param imu IMU sensor for accurate heading information
     * @param numParticles Number of particles to use (default: 500)
     * @param initialFrontLeftReading Initial front left sensor reading for relative measurements
     * @param initialFrontRightReading Initial front right sensor reading for relative measurements
     * @param initialLeftReading Initial left sensor reading for relative measurements
     * @param initialRightReading Initial right sensor reading for relative measurements
     * @param initialHeading Initial IMU heading for relative measurements
     * @param fieldWidth Field width in mm (default: 3658 mm = 12 feet)
     * @param fieldHeight Field height in mm (default: 3658 mm = 12 feet)
     */
    MonteCarloLocalization(
        lemlib::Chassis& chassis,
        pros::Distance& frontDistLeft,
        pros::Distance& frontDistRight,
        pros::Distance& rightDist,
        pros::Distance& leftDist,
        pros::Imu& imu,
        int numParticles = 500,
        double initialFrontLeftReading = 0.0,
        double initialFrontRightReading = 0.0,
        double initialLeftReading = 0.0,
        double initialRightReading = 0.0,
        double initialHeading = 0.0,
        double fieldWidth = 3658.0,
        double fieldHeight = 3658.0
    );
    
    /**
     * @brief Initialize the particle filter
     * 
     * @param initialX Initial X position in mm (relative to start)
     * @param initialY Initial Y position in mm (relative to start)
     * @param initialTheta Initial orientation in radians (relative to start)
     * @param spreadX Standard deviation for X position uncertainty
     * @param spreadY Standard deviation for Y position uncertainty
     * @param spreadTheta Standard deviation for orientation uncertainty
     */
    void initialize(
        double initialX,
        double initialY,
        double initialTheta,
        double spreadX = 50.0,
        double spreadY = 50.0,
        double spreadTheta = 0.1
    );
    
    /**
     * @brief Add a wall to the field simulation
     * 
     * Wall coordinates are relative to the robot's starting position
     * 
     * @param x1 Start X coordinate (relative to start)
     * @param y1 Start Y coordinate (relative to start)
     * @param x2 End X coordinate (relative to start)
     * @param y2 End Y coordinate (relative to start)
     */
    void addWall(double x1, double y1, double x2, double y2);
    
    /**
     * @brief Set up standard VEX field walls
     * 
     * This adds the outer perimeter walls of a standard VEX field
     * Walls are positioned relative to the robot's starting position
     */
    void setupStandardField();
    
    /**
     * @brief Update particle filter with odometry and sensor data
     * 
     * This should be called periodically to update the robot's estimated position
     */
    void update();
    
    /**
     * @brief Get the current best estimate of the robot's position
     * 
     * @return lemlib::Pose The estimated pose (x, y, theta) relative to start
     */
    lemlib::Pose getEstimatedPose() const;
    
    /**
     * @brief Simulate distance sensor readings at a given position
     * 
     * @param x X position in mm (relative to start)
     * @param y Y position in mm (relative to start)
     * @param theta Orientation in radians (relative to initial heading)
     * @param sensorAngle Angle of the sensor relative to robot heading (radians)
     * @param maxDistance Maximum sensing distance (mm)
     * @return double Simulated distance reading (mm)
     */
    double simulateDistanceSensor(
        double x,
        double y,
        double theta,
        double sensorAngle,
        double maxDistance = 2000.0
    ) const;
    
    /**
     * @brief Run a complete simulation of an autonomous path for training
     * 
     * @param path Vector of waypoints (x, y, theta) to follow (relative to start)
     * @param updateCallback Callback function called after each simulation step
     */
    void simulateAutonomousPath(
        const std::vector<lemlib::Pose>& path,
        std::function<void(const lemlib::Pose&, const std::vector<Particle>&)> updateCallback = nullptr
    );
    
    /**
     * @brief Get all current particles (for visualization)
     * 
     * @return const std::vector<Particle>& Reference to all particles
     */
    const std::vector<Particle>& getParticles() const;
    
    /**
     * @brief Get all defined walls (for visualization)
     * 
     * @return const std::vector<Wall>& Reference to all walls
     */
    const std::vector<Wall>& getWalls() const;
    
    /**
     * @brief Reset the particle filter to initial state
     */
    void reset();
    
    /**
     * @brief Set the motion model noise parameters
     * 
     * @param alpha1 Translation noise from translation
     * @param alpha2 Translation noise from rotation
     * @param alpha3 Rotation noise from translation
     * @param alpha4 Rotation noise from rotation
     */
    void setMotionModelParams(double alpha1, double alpha2, double alpha3, double alpha4);
    
    /**
     * @brief Set the sensor model parameters
     * 
     * @param sensorStdDev Standard deviation of sensor measurements (mm)
     * @param maxSensorDistance Maximum reliable sensor distance (mm)
     */
    void setSensorModelParams(double sensorStdDev, double maxSensorDistance);
    
    /**
     * @brief Manually override the current position estimate
     * 
     * @param x X position in mm (relative to start)
     * @param y Y position in mm (relative to start)
     * @param theta Orientation in radians (relative to initial heading)
     */
    void setPosition(double x, double y, double theta);
    
private:
    // References to sensors and chassis
    lemlib::Chassis& m_chassis;
    pros::Distance& m_frontDistLeft;
    pros::Distance& m_frontDistRight;
    pros::Distance& m_rightDist;
    pros::Distance& m_leftDist;
    pros::Imu& m_imu;  // Reference to IMU sensor
    
    // Particles and walls
    std::vector<Particle> m_particles;
    std::vector<Wall> m_walls;
    
    // Field dimensions
    double m_fieldWidth;
    double m_fieldHeight;
    
    // Motion model parameters
    double m_alpha1 = 0.1;  // Translation noise from translation
    double m_alpha2 = 0.1;  // Translation noise from rotation
    double m_alpha3 = 0.1;  // Rotation noise from translation
    double m_alpha4 = 0.1;  // Rotation noise from rotation
    
    // Sensor model parameters
    double m_sensorStdDev = 50.0;        // Standard deviation of sensor noise (mm)
    double m_maxSensorDistance = 2000.0; // Maximum reliable sensor distance (mm)
    
    // Initial sensor readings for relative measurements
    double m_initialFrontLeftReading;
    double m_initialFrontRightReading;
    double m_initialLeftReading;
    double m_initialRightReading;
    
    // Previous odometry readings for motion update
    lemlib::Pose m_prevOdomPose;
    double m_prevImuHeading = 0.0;  // Previous IMU heading in radians
    double m_initialImuHeading = 0.0; // Initial IMU heading for relative heading
    
    // Random number generators
    mutable std::mt19937 m_rng;
    
    // Private methods
    
    /**
     * @brief Update particles based on odometry motion model
     * 
     * @param deltaX Change in X position
     * @param deltaY Change in Y position
     * @param deltaTheta Change in orientation
     */
    void motionUpdate(double deltaX, double deltaY, double deltaTheta);
    
    /**
     * @brief Update particle weights based on sensor measurements
     */
    void sensorUpdate();
    
    /**
     * @brief Resample particles based on their weights
     */
    void resampleParticles();
    
    /**
     * @brief Calculate ray-line segment intersection
     * 
     * @param rayOriginX Ray origin X coordinate
     * @param rayOriginY Ray origin Y coordinate
     * @param rayDirX Ray direction X component
     * @param rayDirY Ray direction Y component
     * @param lineX1 Line segment start X coordinate
     * @param lineY1 Line segment start Y coordinate
     * @param lineX2 Line segment end X coordinate
     * @param lineY2 Line segment end Y coordinate
     * @param intersectionX Output parameter for intersection X coordinate
     * @param intersectionY Output parameter for intersection Y coordinate
     * @return bool True if intersection exists, false otherwise
     */
    bool rayLineIntersection(
        double rayOriginX, double rayOriginY,
        double rayDirX, double rayDirY,
        double lineX1, double lineY1,
        double lineX2, double lineY2,
        double& intersectionX, double& intersectionY
    ) const;
    
    /**
     * @brief Calculate the distance from a point to a line segment
     * 
     * @param pointX Point X coordinate
     * @param pointY Point Y coordinate
     * @param lineX1 Line segment start X coordinate
     * @param lineY1 Line segment start Y coordinate
     * @param lineX2 Line segment end X coordinate
     * @param lineY2 Line segment end Y coordinate
     * @return double Distance from point to line segment
     */
    double pointToLineDistance(
        double pointX, double pointY,
        double lineX1, double lineY1,
        double lineX2, double lineY2
    ) const;
    
    /**
     * @brief Add Gaussian noise to a value
     * 
     * @param value Base value
     * @param stdDev Standard deviation of noise
     * @return double Value with added noise
     */
    double addNoise(double value, double stdDev) const;
    
    /**
     * @brief Get relative distance from a sensor (relative to initial reading)
     * 
     * @param currentReading Current sensor reading
     * @param initialReading Initial sensor reading
     * @return double Relative distance (positive if closer than initial)
     */
    double getRelativeDistance(double currentReading, double initialReading) const;
};

/**
 * @brief Helper class for visualizing the Monte Carlo localization system
 */
class MCLVisualizer {
public:
    /**
     * @brief Constructor
     * 
     * @param mcl Reference to the MonteCarloLocalization instance
     * @param scale Scale factor for visualization (pixels per mm)
     */
    MCLVisualizer(MonteCarloLocalization& mcl, double scale = 0.1);
    
    /**
     * @brief Display the visualization on the PROS screen
     */
    void display();
    
    /**
     * @brief Handle screen touch events for interactive simulation
     */
    void handleTouch();
    
private:
    MonteCarloLocalization& m_mcl;
    double m_scale;
    bool m_simulating = false;
    
    /**
     * @brief Draw a particle
     * 
     * @param particle The particle to draw
     * @param color The color to use
     * @param size The size of the particle dot
     */
    void drawParticle(const MonteCarloLocalization::Particle& particle, uint32_t color, int size);
    
    /**
     * @brief Draw a wall
     * 
     * @param wall The wall to draw
     * @param color The color to use
     */
    void drawWall(const MonteCarloLocalization::Wall& wall, uint32_t color);
    
    /**
     * @brief Draw the estimated robot position
     * 
     * @param pose The estimated pose
     * @param color The color to use
     */
    void drawRobot(const lemlib::Pose& pose, uint32_t color);
}; 
