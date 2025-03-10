#include "bennyHeaders/MonteCarloLocalization.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <random>
#include <chrono>
#include "display/lvgl.h"

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

// Normalize angle to [-π, π]
double normalizeAngle(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Constructor
MonteCarloLocalization::MonteCarloLocalization(
    lemlib::Chassis& chassis,
    pros::Distance& frontDistLeft,
    pros::Distance& frontDistRight,
    pros::Distance& rightDist,
    pros::Distance& leftDist,
    pros::Imu& imu,
    int numParticles,
    double initialFrontLeftReading,
    double initialFrontRightReading,
    double initialLeftReading,
    double initialRightReading,
    double initialHeading,
    double fieldWidth,
    double fieldHeight
) : m_chassis(chassis),
    m_frontDistLeft(frontDistLeft),
    m_frontDistRight(frontDistRight),
    m_rightDist(rightDist),
    m_leftDist(leftDist),
    m_imu(imu),
    m_initialFrontLeftReading(initialFrontLeftReading),
    m_initialFrontRightReading(initialFrontRightReading),
    m_initialLeftReading(initialLeftReading),
    m_initialRightReading(initialRightReading),
    m_initialImuHeading(initialHeading * DEG_TO_RAD),
    m_fieldWidth(fieldWidth),
    m_fieldHeight(fieldHeight),
    m_rng(std::chrono::system_clock::now().time_since_epoch().count())
{
    // Resize particles vector
    m_particles.resize(numParticles);
    
    // Initialize previous odometry pose
    m_prevOdomPose = m_chassis.getPose();
    
    // Initialize previous IMU heading
    m_prevImuHeading = m_imu.get_heading() * DEG_TO_RAD;
}

// Initialize the particle filter with particles around a given position
void MonteCarloLocalization::initialize(
    double initialX,
    double initialY,
    double initialTheta,
    double spreadX,
    double spreadY,
    double spreadTheta
) {
    // Create normal distributions for position and orientation
    std::normal_distribution<double> distX(initialX, spreadX);
    std::normal_distribution<double> distY(initialY, spreadY);
    std::normal_distribution<double> distTheta(initialTheta, spreadTheta);
    
    // Initialize particles with random positions around the initial position
    for (auto& particle : m_particles) {
        particle.x = distX(m_rng);
        particle.y = distY(m_rng);
        particle.theta = distTheta(m_rng);
        particle.weight = 1.0 / m_particles.size();
    }
    
    // Set previous odometry pose
    m_prevOdomPose = m_chassis.getPose();
    
    // Reset previous IMU heading
    m_prevImuHeading = m_imu.get_heading() * DEG_TO_RAD;
}

// Add a wall to the field simulation
void MonteCarloLocalization::addWall(double x1, double y1, double x2, double y2) {
    m_walls.emplace_back(x1, y1, x2, y2);
}

// Set up standard VEX field walls relative to the robot's starting position
void MonteCarloLocalization::setupStandardField() {
    // Clear existing walls
    m_walls.clear();
    
    // Create walls based on initial sensor readings
    // This is a simplified approximation that assumes the robot is somewhat
    // parallel to the field walls
    
    // Estimated field boundaries (relative to robot's starting position)
    double fieldLeft = -m_initialLeftReading;
    double fieldRight = m_initialRightReading;
    double fieldFront = (m_initialFrontLeftReading + m_initialFrontRightReading) / 2.0;
    double fieldBack = -1000; // Arbitrary back wall distance
    
    // Add outer perimeter walls (all coordinates relative to robot start position)
    addWall(fieldLeft, fieldBack, fieldRight, fieldBack);  // Back wall
    addWall(fieldRight, fieldBack, fieldRight, fieldFront); // Right wall
    addWall(fieldRight, fieldFront, fieldLeft, fieldFront); // Front wall
    addWall(fieldLeft, fieldFront, fieldLeft, fieldBack);   // Left wall
}

// Update the particle filter
void MonteCarloLocalization::update() {
    // Get current odometry reading
    lemlib::Pose currentOdomPose = m_chassis.getPose();
    
    // Calculate change in position and orientation
    double deltaX = currentOdomPose.x - m_prevOdomPose.x;
    double deltaY = currentOdomPose.y - m_prevOdomPose.y;
    double deltaTheta = normalizeAngle(currentOdomPose.theta * DEG_TO_RAD - m_prevOdomPose.theta * DEG_TO_RAD);
    
    // Update particles based on motion model
    motionUpdate(deltaX, deltaY, deltaTheta);
    
    // Update particle weights based on sensor readings
    sensorUpdate();
    
    // Resample particles if needed
    resampleParticles();
    
    // Save current odometry reading for next update
    m_prevOdomPose = currentOdomPose;
    
    // Save current IMU heading for next update
    m_prevImuHeading = m_imu.get_heading() * DEG_TO_RAD;
}

// Get the current best estimate of the robot's position
lemlib::Pose MonteCarloLocalization::getEstimatedPose() const {
    // Find particle with highest weight
    auto bestParticle = std::max_element(
        m_particles.begin(),
        m_particles.end(),
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; }
    );
    
    // Return the position and orientation of the best particle
    return lemlib::Pose(
        bestParticle->x,
        bestParticle->y,
        normalizeAngle(bestParticle->theta) * RAD_TO_DEG
    );
}

// Get relative distance measurement (current vs initial)
double MonteCarloLocalization::getRelativeDistance(double currentReading, double initialReading) const {
    // Calculate relative distance (positive = closer than initial, negative = farther than initial)
    return initialReading - currentReading;
}

// Simulate distance sensor readings at a given position
double MonteCarloLocalization::simulateDistanceSensor(
    double x,
    double y,
    double theta,
    double sensorAngle,
    double maxDistance
) const {
    // Calculate ray direction
    double rayAngle = theta + sensorAngle;
    double rayDirX = std::cos(rayAngle);
    double rayDirY = std::sin(rayAngle);
    
    // Find closest intersection with any wall
    double minDistance = maxDistance;
    
    for (const auto& wall : m_walls) {
        double intersectionX, intersectionY;
        
        if (rayLineIntersection(
                x, y,
                rayDirX, rayDirY,
                wall.x1, wall.y1,
                wall.x2, wall.y2,
                intersectionX, intersectionY
            )) {
            
            // Calculate distance to intersection
            double dx = intersectionX - x;
            double dy = intersectionY - y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            // Update minimum distance if this intersection is closer
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }
    
    // Add some noise to the simulated reading
    return addNoise(minDistance, m_sensorStdDev);
}

// Run a complete simulation of an autonomous path
void MonteCarloLocalization::simulateAutonomousPath(
    const std::vector<lemlib::Pose>& path,
    std::function<void(const lemlib::Pose&, const std::vector<Particle>&)> updateCallback
) {
    if (path.empty()) return;
    
    // Initialize the particle filter at the starting position
    initialize(
        path[0].x,
        path[0].y, 
        path[0].theta * DEG_TO_RAD,
        10.0, 10.0, 0.05 // Small initial uncertainty
    );
    
    // Start with the first waypoint as the current position
    lemlib::Pose currentPose = path[0];
    
    // Simulate movement between waypoints
    for (size_t i = 1; i < path.size(); ++i) {
        // Calculate the direction and distance to the next waypoint
        double targetX = path[i].x;
        double targetY = path[i].y;
        double targetTheta = path[i].theta * DEG_TO_RAD;
        
        // Simulate movement in small steps
        const int numSteps = 20;
        double dx = (targetX - currentPose.x) / numSteps;
        double dy = (targetY - currentPose.y) / numSteps;
        double dtheta = normalizeAngle(targetTheta - currentPose.theta * DEG_TO_RAD) / numSteps;
        
        for (int step = 0; step < numSteps; ++step) {
            // Update current position
            currentPose.x += dx;
            currentPose.y += dy;
            currentPose.theta += dtheta * RAD_TO_DEG;
            
            // Update particle filter with this motion
            motionUpdate(dx, dy, dtheta);
            
            // Simulate sensor readings and update particles
            sensorUpdate();
            
            // Resample particles
            resampleParticles();
            
            // Call the update callback if provided
            if (updateCallback) {
                updateCallback(currentPose, m_particles);
            }
            
            // Short delay for visualization
            pros::delay(10);
        }
    }
}

// Get all current particles
const std::vector<MonteCarloLocalization::Particle>& MonteCarloLocalization::getParticles() const {
    return m_particles;
}

// Get all defined walls
const std::vector<MonteCarloLocalization::Wall>& MonteCarloLocalization::getWalls() const {
    return m_walls;
}

// Reset the particle filter
void MonteCarloLocalization::reset() {
    initialize(
        m_prevOdomPose.x,
        m_prevOdomPose.y,
        m_prevOdomPose.theta * DEG_TO_RAD,
        100.0, 100.0, 0.2 // Large uncertainty
    );
}

// Set the motion model noise parameters
void MonteCarloLocalization::setMotionModelParams(
    double alpha1, double alpha2, double alpha3, double alpha4
) {
    m_alpha1 = alpha1;
    m_alpha2 = alpha2;
    m_alpha3 = alpha3;
    m_alpha4 = alpha4;
}

// Set the sensor model parameters
void MonteCarloLocalization::setSensorModelParams(
    double sensorStdDev, double maxSensorDistance
) {
    m_sensorStdDev = sensorStdDev;
    m_maxSensorDistance = maxSensorDistance;
}

// Override the current position estimate
void MonteCarloLocalization::setPosition(double x, double y, double theta) {
    // Reset particles with very small spread around the given position
    initialize(x, y, theta, 5.0, 5.0, 0.02);
}

// Private Methods

// Update particles based on odometry motion model
void MonteCarloLocalization::motionUpdate(double deltaX, double deltaY, double deltaTheta) {
    // Get current IMU heading in radians
    double imuHeading = m_imu.get_heading() * DEG_TO_RAD;
    // Convert to relative heading (relative to initial heading)
    double relativeImuHeading = normalizeAngle(imuHeading - m_initialImuHeading);
    
    // Calculate heading change from IMU
    double imuDeltaTheta = normalizeAngle(relativeImuHeading - m_prevImuHeading);
    
    // If IMU delta is valid (not a huge jump due to reset), use it instead of odometry
    if (std::abs(imuDeltaTheta) < PI / 2) {
        deltaTheta = imuDeltaTheta;
    }
    
    // Calculate translational and rotational components
    double trans = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    double rot1 = std::atan2(deltaY, deltaX) - m_prevOdomPose.theta * DEG_TO_RAD;
    double rot2 = deltaTheta - rot1;
    
    // Calculate standard deviations for noise
    double transSigma = std::sqrt(m_alpha1 * trans * trans + m_alpha2 * deltaTheta * deltaTheta);
    double rot1Sigma = std::sqrt(m_alpha3 * trans * trans + m_alpha4 * rot1 * rot1);
    double rot2Sigma = std::sqrt(m_alpha3 * trans * trans + m_alpha4 * rot2 * rot2);
    
    // Create noise distributions
    std::normal_distribution<double> transNoise(0.0, transSigma);
    std::normal_distribution<double> rot1Noise(0.0, rot1Sigma);
    std::normal_distribution<double> rot2Noise(0.0, rot2Sigma);
    
    // Update each particle
    for (auto& particle : m_particles) {
        // Add noise to motion
        double noisyRot1 = normalizeAngle(rot1 + rot1Noise(m_rng));
        double noisyTrans = trans + transNoise(m_rng);
        double noisyRot2 = normalizeAngle(rot2 + rot2Noise(m_rng));
        
        // Apply motion model
        particle.x += noisyTrans * std::cos(particle.theta + noisyRot1);
        particle.y += noisyTrans * std::sin(particle.theta + noisyRot1);
        particle.theta = normalizeAngle(particle.theta + noisyRot1 + noisyRot2);
    }
}

// Update particle weights based on sensor measurements
void MonteCarloLocalization::sensorUpdate() {
    // Get actual sensor readings
    double frontLeftReading = m_frontDistLeft.get();
    double frontRightReading = m_frontDistRight.get();
    double rightReading = m_rightDist.get();
    double leftReading = m_leftDist.get();
    
    // Convert to relative readings (positive = closer than initial, negative = farther)
    double relFrontLeftReading = getRelativeDistance(frontLeftReading, m_initialFrontLeftReading);
    double relFrontRightReading = getRelativeDistance(frontRightReading, m_initialFrontRightReading);
    double relRightReading = getRelativeDistance(rightReading, m_initialRightReading);
    double relLeftReading = getRelativeDistance(leftReading, m_initialLeftReading);
    
    // Get IMU heading for measurement update (relative to initial heading)
    double imuHeading = normalizeAngle(m_imu.get_heading() * DEG_TO_RAD - m_initialImuHeading);
    
    // Define sensor angles relative to robot heading
    // Assuming front sensors are angled slightly outward (adjust based on your robot)
    const double frontLeftAngle = -PI/8;  // -22.5 degrees
    const double frontRightAngle = PI/8;  // 22.5 degrees
    const double rightAngle = PI/2;       // 90 degrees
    const double leftAngle = -PI/2;       // -90 degrees
    
    // Update weights of all particles
    for (auto& particle : m_particles) {
        // Simulate sensor readings for this particle
        double simFrontLeftReading = simulateDistanceSensor(
            particle.x, particle.y, particle.theta, frontLeftAngle);
        double simFrontRightReading = simulateDistanceSensor(
            particle.x, particle.y, particle.theta, frontRightAngle);
        double simRightReading = simulateDistanceSensor(
            particle.x, particle.y, particle.theta, rightAngle);
        double simLeftReading = simulateDistanceSensor(
            particle.x, particle.y, particle.theta, leftAngle);
        
        // Calculate likelihoods based on the difference between simulated and real RELATIVE readings
        double frontLeftLikelihood = 1.0;
        double frontRightLikelihood = 1.0;
        double rightLikelihood = 1.0;
        double leftLikelihood = 1.0;
        
        // Only use readings within valid range
        if (frontLeftReading <= m_maxSensorDistance && simFrontLeftReading <= m_maxSensorDistance) {
            // Compare relative values (how much the reading has changed from initial)
            double error = relFrontLeftReading - (m_initialFrontLeftReading - simFrontLeftReading);
            frontLeftLikelihood = std::exp(-(error*error) / (2 * m_sensorStdDev * m_sensorStdDev));
        }
        
        if (frontRightReading <= m_maxSensorDistance && simFrontRightReading <= m_maxSensorDistance) {
            double error = relFrontRightReading - (m_initialFrontRightReading - simFrontRightReading);
            frontRightLikelihood = std::exp(-(error*error) / (2 * m_sensorStdDev * m_sensorStdDev));
        }
        
        if (rightReading <= m_maxSensorDistance && simRightReading <= m_maxSensorDistance) {
            double error = relRightReading - (m_initialRightReading - simRightReading);
            rightLikelihood = std::exp(-(error*error) / (2 * m_sensorStdDev * m_sensorStdDev));
        }
        
        if (leftReading <= m_maxSensorDistance && simLeftReading <= m_maxSensorDistance) {
            double error = relLeftReading - (m_initialLeftReading - simLeftReading);
            leftLikelihood = std::exp(-(error*error) / (2 * m_sensorStdDev * m_sensorStdDev));
        }
        
        // Calculate heading error and likelihood from IMU
        double headingError = normalizeAngle(particle.theta - imuHeading);
        
        // IMU is typically more accurate than distance sensors, use smaller stddev
        const double headingStdDev = 0.05; // Radians (about 3 degrees)
        double headingLikelihood = std::exp(-(headingError*headingError) / 
                                         (2 * headingStdDev * headingStdDev));
        
        // Apply a higher weight to IMU measurements (they're more reliable)
        headingLikelihood = std::pow(headingLikelihood, 2.0);
        
        // Combine likelihoods (product of all likelihoods)
        particle.weight *= frontLeftLikelihood * frontRightLikelihood * 
                          rightLikelihood * leftLikelihood * headingLikelihood;
    }
    
    // Normalize weights
    double sumWeights = 0.0;
    for (const auto& particle : m_particles) {
        sumWeights += particle.weight;
    }
    
    if (sumWeights > 0.0) {
        for (auto& particle : m_particles) {
            particle.weight /= sumWeights;
        }
    } else {
        // If all weights are zero, reset to uniform weights
        double uniformWeight = 1.0 / m_particles.size();
        for (auto& particle : m_particles) {
            particle.weight = uniformWeight;
        }
    }
}

// Resample particles based on their weights
void MonteCarloLocalization::resampleParticles() {
    // Calculate effective number of particles
    double sumSquaredWeights = 0.0;
    for (const auto& particle : m_particles) {
        sumSquaredWeights += particle.weight * particle.weight;
    }
    
    double effectiveN = 1.0 / sumSquaredWeights;
    double threshold = 0.5 * m_particles.size();
    
    // Only resample if effective number of particles is below threshold
    if (effectiveN < threshold) {
        // Create a copy of the current particles
        std::vector<Particle> oldParticles = m_particles;
        
        // Create cumulative distribution for sampling
        std::vector<double> cumWeights(oldParticles.size());
        cumWeights[0] = oldParticles[0].weight;
        for (size_t i = 1; i < oldParticles.size(); ++i) {
            cumWeights[i] = cumWeights[i-1] + oldParticles[i].weight;
        }
        
        // Resample with replacement using low-variance sampler
        std::uniform_real_distribution<double> uniformDist(0.0, 1.0 / m_particles.size());
        double r = uniformDist(m_rng);
        int idx = 0;
        
        for (size_t i = 0; i < m_particles.size(); ++i) {
            double u = r + i / (double)m_particles.size();
            while (u > cumWeights[idx] && idx < oldParticles.size() - 1) {
                idx++;
            }
            
            // Copy the selected particle
            m_particles[i] = oldParticles[idx];
            m_particles[i].weight = 1.0 / m_particles.size();
            
            // Add a small amount of noise to prevent particle depletion
            m_particles[i].x = addNoise(m_particles[i].x, 1.0);
            m_particles[i].y = addNoise(m_particles[i].y, 1.0);
            m_particles[i].theta = addNoise(m_particles[i].theta, 0.01);
        }
    }
}

// Calculate ray-line segment intersection
bool MonteCarloLocalization::rayLineIntersection(
    double rayOriginX, double rayOriginY,
    double rayDirX, double rayDirY,
    double lineX1, double lineY1,
    double lineX2, double lineY2,
    double& intersectionX, double& intersectionY
) const {
    // Calculate line segment direction
    double lineDirectionX = lineX2 - lineX1;
    double lineDirectionY = lineY2 - lineY1;
    
    // Calculate determinant
    double det = rayDirX * lineDirectionY - rayDirY * lineDirectionX;
    
    // Check if ray and line are parallel
    if (std::abs(det) < 1e-6) {
        return false;
    }
    
    // Calculate parameters t and u
    double dx = lineX1 - rayOriginX;
    double dy = lineY1 - rayOriginY;
    
    double t = (dx * lineDirectionY - dy * lineDirectionX) / det;
    double u = (dx * rayDirY - dy * rayDirX) / det;
    
    // Check if intersection is valid
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
        // Calculate intersection point
        intersectionX = rayOriginX + t * rayDirX;
        intersectionY = rayOriginY + t * rayDirY;
        return true;
    }
    
    return false;
}

// Calculate the distance from a point to a line segment
double MonteCarloLocalization::pointToLineDistance(
    double pointX, double pointY,
    double lineX1, double lineY1,
    double lineX2, double lineY2
) const {
    // Calculate squared length of line segment
    double lineLengthSq = (lineX2 - lineX1) * (lineX2 - lineX1) + 
                          (lineY2 - lineY1) * (lineY2 - lineY1);
    
    // If the line segment is just a point, return distance to that point
    if (lineLengthSq < 1e-6) {
        double dx = pointX - lineX1;
        double dy = pointY - lineY1;
        return std::sqrt(dx*dx + dy*dy);
    }
    
    // Calculate projection of point onto line segment
    double t = ((pointX - lineX1) * (lineX2 - lineX1) + 
                (pointY - lineY1) * (lineY2 - lineY1)) / lineLengthSq;
    
    // Clamp t to [0, 1] for the line segment
    t = std::max(0.0, std::min(1.0, t));
    
    // Calculate closest point on line segment
    double closestX = lineX1 + t * (lineX2 - lineX1);
    double closestY = lineY1 + t * (lineY2 - lineY1);
    
    // Calculate distance to closest point
    double dx = pointX - closestX;
    double dy = pointY - closestY;
    return std::sqrt(dx*dx + dy*dy);
}

// Add Gaussian noise to a value
double MonteCarloLocalization::addNoise(double value, double stdDev) const {
    if (stdDev <= 0.0) {
        return value;
    }
    
    std::normal_distribution<double> dist(0.0, stdDev);
    return value + dist(m_rng);
}

// MCLVisualizer implementation

MCLVisualizer::MCLVisualizer(MonteCarloLocalization& mcl, double scale)
    : m_mcl(mcl), m_scale(scale) {
}

void MCLVisualizer::display() {
    // Clear the screen
    lv_obj_t* scr = lv_scr_act();
    lv_obj_clean(scr);
    
    // Get particles and walls
    const auto& particles = m_mcl.getParticles();
    const auto& walls = m_mcl.getWalls();
    
    // Draw walls
    for (const auto& wall : walls) {
        drawWall(wall, LV_COLOR_WHITE.full);
    }
    
    // Draw particles (a subsample for performance)
    const int particleSampleRate = std::max(1, static_cast<int>(particles.size() / 100));
    for (size_t i = 0; i < particles.size(); i += particleSampleRate) {
        uint32_t weight = static_cast<uint32_t>(255 * particles[i].weight * particles.size());
        weight = std::min(weight, 255u);
        
        lv_color_t color = lv_color_make(255, weight, weight);
        drawParticle(particles[i], color.full, 2);
    }
    
    // Draw estimated robot position
    lemlib::Pose pose = m_mcl.getEstimatedPose();
    drawRobot(pose, LV_COLOR_GREEN.full);
    
    // Display current position information
    char posText[100];
    sprintf(posText, "Pos: %.1f, %.1f, %.1f°", pose.x, pose.y, pose.theta);
    
    lv_obj_t* labelPos = lv_label_create(scr, NULL);
    lv_label_set_text(labelPos, posText);
    lv_obj_align(labelPos, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, -10);
    
    // Display simulation controls if needed
    if (m_simulating) {
        lv_obj_t* simLabel = lv_label_create(scr, NULL);
        lv_label_set_text(simLabel, "SIMULATING...");
        lv_obj_align(simLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);
    } else {
        // Create simulation button
        lv_obj_t* simBtn = lv_btn_create(scr, NULL);
        lv_obj_set_size(simBtn, 100, 40);
        lv_obj_align(simBtn, NULL, LV_ALIGN_IN_TOP_RIGHT, -10, 10);
        
        lv_obj_t* simLabel = lv_label_create(simBtn, NULL);
        lv_label_set_text(simLabel, "Simulate");
        
        // Create reset button
        lv_obj_t* resetBtn = lv_btn_create(scr, NULL);
        lv_obj_set_size(resetBtn, 100, 40);
        lv_obj_align(resetBtn, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
        
        lv_obj_t* resetLabel = lv_label_create(resetBtn, NULL);
        lv_label_set_text(resetLabel, "Reset");
    }
}

void MCLVisualizer::handleTouch() {
    lv_indev_t* indev = lv_indev_get_act();
    if (indev == NULL) return;
    
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    
    // Check if a button was pressed
    lv_obj_t* obj = lv_indev_search_obj(lv_scr_act(), &point);
    if (obj != NULL) {
        lv_obj_t* label = lv_obj_get_child(obj, NULL);
        if (label != NULL) {
            const char* text = lv_label_get_text(label);
            
            if (strcmp(text, "Simulate") == 0) {
                m_simulating = true;
                display();
                
                // Set up a simple path to simulate around the starting point
                std::vector<lemlib::Pose> path;
                path.push_back(lemlib::Pose(0, 0, 0));         // Start position
                path.push_back(lemlib::Pose(300, 0, 90));      // Forward then turn
                path.push_back(lemlib::Pose(300, 300, 180));   // Right then turn
                path.push_back(lemlib::Pose(0, 300, 270));     // Back then turn
                path.push_back(lemlib::Pose(0, 0, 0));         // Left then return to start
                
                // Run simulation with visualization callback
                m_mcl.simulateAutonomousPath(path, [this](const lemlib::Pose& pose, const std::vector<MonteCarloLocalization::Particle>& particles) {
                    display();
                });
                
                m_simulating = false;
                display();
            } else if (strcmp(text, "Reset") == 0) {
                m_mcl.reset();
                display();
            }
        }
    }
}

void MCLVisualizer::drawParticle(const MonteCarloLocalization::Particle& particle, uint32_t color, int size) {
    // Center the display on (0,0) which is the robot's starting position
    const int centerX = 240;
    const int centerY = 240;
    
    int x = centerX + static_cast<int>(particle.x * m_scale);
    int y = centerY - static_cast<int>(particle.y * m_scale); // Y is inverted in screen coordinates
    
    lv_obj_t* point = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(point, size, size);
    lv_obj_set_pos(point, x, y);
    lv_obj_set_style_local_bg_color(point, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(color));
    lv_obj_set_style_local_border_width(point, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
}

void MCLVisualizer::drawWall(const MonteCarloLocalization::Wall& wall, uint32_t color) {
    // Center the display on (0,0) which is the robot's starting position
    const int centerX = 240;
    const int centerY = 240;
    
    int x1 = centerX + static_cast<int>(wall.x1 * m_scale);
    int y1 = centerY - static_cast<int>(wall.y1 * m_scale); // Y is inverted
    int x2 = centerX + static_cast<int>(wall.x2 * m_scale);
    int y2 = centerY - static_cast<int>(wall.y2 * m_scale); // Y is inverted
    
    lv_point_t points[] = {{x1, y1}, {x2, y2}};
    
    // Create a line object
    lv_obj_t* line = lv_line_create(lv_scr_act(), NULL);
    lv_line_set_points(line, points, 2);
    lv_obj_set_style_local_line_color(line, LV_LINE_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(color));
    lv_obj_set_style_local_line_width(line, LV_LINE_PART_MAIN, LV_STATE_DEFAULT, 2);
}

void MCLVisualizer::drawRobot(const lemlib::Pose& pose, uint32_t color) {
    // Center the display on (0,0) which is the robot's starting position
    const int centerX = 240;
    const int centerY = 240;
    
    int x = centerX + static_cast<int>(pose.x * m_scale);
    int y = centerY - static_cast<int>(pose.y * m_scale); // Y is inverted
    double theta = pose.theta * DEG_TO_RAD;
    
    // Robot body (a circle)
    const int robotRadius = 15;
    
    lv_obj_t* robot = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(robot, robotRadius * 2, robotRadius * 2);
    lv_obj_set_pos(robot, x - robotRadius, y - robotRadius);
    lv_obj_set_style_local_radius(robot, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, robotRadius);
    lv_obj_set_style_local_bg_color(robot, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, lv_color_hex(color));
    lv_obj_set_style_local_border_width(robot, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 2);
    lv_obj_set_style_local_border_color(robot, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    
    // Direction indicator (a line)
    int dirX = x + static_cast<int>(robotRadius * 1.5 * cos(theta));
    int dirY = y - static_cast<int>(robotRadius * 1.5 * sin(theta)); // Y is inverted
    
    lv_point_t points[] = {{x, y}, {dirX, dirY}};
    lv_obj_t* line = lv_line_create(lv_scr_act(), NULL);
    lv_line_set_points(line, points, 2);
    lv_obj_set_style_local_line_color(line, LV_LINE_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_set_style_local_line_width(line, LV_LINE_PART_MAIN, LV_STATE_DEFAULT, 2);
} 
