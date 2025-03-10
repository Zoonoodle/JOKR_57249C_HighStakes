#pragma once

#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include <vector>
#include <string>
#include <fstream>
#include <ctime>

/**
 * @brief Data frame containing robot state and sensor readings at a point in time
 */
struct SensorFrame {
    uint32_t timestamp;      // Time since recording started (ms)
    double x;                // X position relative to start (mm)
    double y;                // Y position relative to start (mm)
    double heading;          // Heading in degrees
    double frontLeftDist;    // Front left distance sensor reading (mm)
    double frontRightDist;   // Front right distance sensor reading (mm)
    double leftDist;         // Left distance sensor reading (mm)
    double rightDist;        // Right distance sensor reading (mm)
    double imuReading;       // IMU heading in degrees
};

/**
 * @brief Class for recording and playing back robot routes with sensor data
 * 
 * This class allows recording the robot's movement during driver control,
 * saving it to the SD card, and playing it back for MCL testing and visualization.
 */
class RouteRecorder {
public:
    /**
     * @brief Constructor for RouteRecorder
     */
    RouteRecorder();
    
    /**
     * @brief Begin recording robot movement and sensor data
     * 
     * Records odometry position, sensor readings, and IMU heading at regular intervals.
     */
    void startRecording();
    
    /**
     * @brief Stop the current recording
     */
    void stopRecording();
    
    /**
     * @brief Save the recorded data to the SD card
     * 
     * @param filename Name of the file to save (will be saved to /usd/)
     * @return bool True if saved successfully
     */
    bool saveToSD(const std::string& filename);
    
    /**
     * @brief Play back a previously recorded route using MCL
     * 
     * @param filename Name of the file to load
     * @param visualizeMCL Whether to show MCL visualization during playback
     * @return bool True if playback was successful
     */
    bool playbackFromSD(const std::string& filename, bool visualizeMCL = true);
    
    /**
     * @brief Get a list of available recordings on the SD card
     * 
     * @return std::vector<std::string> List of route filenames
     */
    std::vector<std::string> listRecordings();
    
    /**
     * @brief Check if a recording is currently in progress
     * 
     * @return bool True if recording
     */
    bool isRecording() const { return m_recording; }
    
    /**
     * @brief Get the total frames in the current recording
     * 
     * @return size_t Number of frames
     */
    size_t getFrameCount() const { return m_frames.size(); }
    
    /**
     * @brief Add a frame to the recording
     * 
     * @param frame The frame to add
     */
    void addFrame(const SensorFrame& frame);
    
    /**
     * @brief Get the sample rate in milliseconds
     * 
     * @return int Sample rate in ms
     */
    int getSampleRate() const;
    
    // Make the start time accessible to the task
    uint32_t m_startTime = 0;
    
private:
    // The recorded frames
    std::vector<SensorFrame> m_frames;
    
    // Whether recording is in progress
    bool m_recording = false;
    
    // Task handle for recording
    pros::Task* m_recordTask = nullptr;
    
    // Recording sample rate in ms
    int m_sampleRateMs = 100;
};

// Global recorder instance
extern RouteRecorder recorder; 