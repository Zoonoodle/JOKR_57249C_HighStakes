#include "bennyHeaders/RouteRecorder.hpp"
#include "bennyHeaders/MCLSystem.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "pros/apix.h"
#include <dirent.h>
#include <sys/stat.h>
#include <algorithm>
#include <iomanip>
#include <sstream>

// Define some constants
constexpr double DEG_TO_RAD = 0.01745329251;
constexpr double RAD_TO_DEG = 57.2957795131;

// Global recorder instance
RouteRecorder recorder;

// Task function for recording
void recordingTaskFn(void* param) {
    RouteRecorder* recorder = static_cast<RouteRecorder*>(param);
    if (!recorder) return;
    
    // Record frames as long as recording is active
    while (recorder->isRecording()) {
        // Create a new frame with current data
        SensorFrame frame;
        frame.timestamp = pros::millis() - recorder->m_startTime;
        
        // Get current position
        lemlib::Pose pose = chassis.getPose();
        frame.x = pose.x;
        frame.y = pose.y;
        frame.heading = pose.theta;
        
        // Get sensor readings
        frame.frontLeftDist = frontDistLeft.get();
        frame.frontRightDist = frontDistRight.get();
        frame.leftDist = leftDist.get();
        frame.rightDist = rightDist.get();
        
        // Get IMU reading
        frame.imuReading = imu.get_heading();
        
        // Add frame to recording
        recorder->addFrame(frame);
        
        // Sample at requested rate
        pros::delay(recorder->getSampleRate());
    }
}

RouteRecorder::RouteRecorder() : m_recordTask(nullptr) {
    // Initialize with reasonable default sample rate
    m_sampleRateMs = 100; // 10Hz recording
}

void RouteRecorder::startRecording() {
    // Clear previous recording if any
    m_frames.clear();
    
    // Start recording
    m_recording = true;
    m_startTime = pros::millis();
    
    // Create recording task
    if (m_recordTask != nullptr) {
        m_recordTask->remove();
        delete m_recordTask;
    }
    
    m_recordTask = new pros::Task(recordingTaskFn, this, "Recording Task");
    
    // Show status on screen
    pros::lcd::print(0, "Recording started");
}

void RouteRecorder::stopRecording() {
    // Stop recording
    m_recording = false;
    
    // Wait for task to finish
    if (m_recordTask != nullptr) {
        pros::delay(m_sampleRateMs * 2); // Give task time to finish
        m_recordTask->remove();
        delete m_recordTask;
        m_recordTask = nullptr;
    }
    
    // Show status
    pros::lcd::print(0, "Recording stopped: %d frames", m_frames.size());
}

bool RouteRecorder::saveToSD(const std::string& filename) {
    // Make sure we have data to save
    if (m_frames.empty()) {
        pros::lcd::print(1, "No recording to save");
        return false;
    }
    
    // Create full filepath
    std::string filepath = "/usd/" + filename;
    
    // Add file extension if not provided
    if (filepath.find(".route") == std::string::npos) {
        filepath += ".route";
    }
    
    // Open file for writing
    std::ofstream file(filepath.c_str(), std::ios::binary | std::ios::out);
    
    if (!file.is_open()) {
        pros::lcd::print(1, "Failed to open %s", filepath.c_str());
        return false;
    }
    
    // Write header with version info and frame count
    const uint32_t VERSION = 1;
    file.write(reinterpret_cast<const char*>(&VERSION), sizeof(VERSION));
    
    // Write number of frames
    size_t frameCount = m_frames.size();
    file.write(reinterpret_cast<const char*>(&frameCount), sizeof(frameCount));
    
    // Write sample rate
    file.write(reinterpret_cast<const char*>(&m_sampleRateMs), sizeof(m_sampleRateMs));
    
    // Write all frames
    file.write(reinterpret_cast<const char*>(m_frames.data()), 
               m_frames.size() * sizeof(SensorFrame));
    
    file.close();
    
    pros::lcd::print(1, "Saved %d frames to %s", frameCount, filepath.c_str());
    
    return true;
}

bool RouteRecorder::playbackFromSD(const std::string& filename, bool visualizeMCL) {
    // Create full filepath
    std::string filepath = "/usd/" + filename;
    
    // Add file extension if not provided
    if (filepath.find(".route") == std::string::npos) {
        filepath += ".route";
    }
    
    // Open file for reading
    std::ifstream file(filepath.c_str(), std::ios::binary | std::ios::in);
    
    if (!file.is_open()) {
        pros::lcd::print(1, "Failed to open %s", filepath.c_str());
        return false;
    }
    
    // Read header with version info
    uint32_t version;
    file.read(reinterpret_cast<char*>(&version), sizeof(version));
    
    if (version != 1) {
        pros::lcd::print(1, "Unsupported file version: %d", version);
        file.close();
        return false;
    }
    
    // Read number of frames
    size_t frameCount;
    file.read(reinterpret_cast<char*>(&frameCount), sizeof(frameCount));
    
    // Read sample rate
    int sampleRate;
    file.read(reinterpret_cast<char*>(&sampleRate), sizeof(sampleRate));
    
    // Allocate memory for frames
    std::vector<SensorFrame> playbackFrames(frameCount);
    
    // Read all frames
    file.read(reinterpret_cast<char*>(playbackFrames.data()),
             frameCount * sizeof(SensorFrame));
    
    file.close();
    
    pros::lcd::print(1, "Loaded %d frames from %s", frameCount, filepath.c_str());
    
    // Initialize MCL if visualization is requested
    if (visualizeMCL && !playbackFrames.empty()) {
        // Get initial frame
        const SensorFrame& initialFrame = playbackFrames[0];
        
        // Initialize MCL with initial heading
        // (0,0 will be the starting position in relative coordinates)
        initializeMCL(initialFrame.heading);
        
        // Display initial state
        displayMCL();
    }
    
    // Show playback status
    pros::lcd::print(0, "Playing back %s", filename.c_str());
    
    // Playback all frames
    uint32_t playbackStartTime = pros::millis();
    
    for (size_t i = 0; i < playbackFrames.size(); i++) {
        const SensorFrame& frame = playbackFrames[i];
        
        // Calculate elapsed time in playback
        uint32_t targetTime = frame.timestamp;
        uint32_t currentTime = pros::millis() - playbackStartTime;
        
        // Wait until the right time for this frame
        if (targetTime > currentTime) {
            pros::delay(targetTime - currentTime);
        }
        
        // Update MCL with this frame's data if visualizing
        if (visualizeMCL) {
            // Set MCL position based on recorded data
            setMCLPosition(frame.x, frame.y, frame.heading);
            
            // Alternative: We could also use recorded sensor readings to update MCL
            // This would test the actual MCL algorithm with real data
            
            // Update visualization
            displayMCL();
        }
        
        // Display playback progress
        pros::lcd::print(2, "Frame %d/%d: %.1f, %.1f, %.1f°", 
                        i+1, playbackFrames.size(),
                        frame.x, frame.y, frame.heading);
    }
    
    pros::lcd::print(0, "Playback complete");
    
    return true;
}

std::vector<std::string> RouteRecorder::listRecordings() {
    std::vector<std::string> fileList;
    
    // Open SD card directory
    DIR* dir = opendir("/usd");
    
    if (dir == nullptr) {
        pros::lcd::print(1, "Failed to open /usd directory");
        return fileList;
    }
    
    // Read all files in directory
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        
        // Check if this is a route file
        if (filename.size() > 6 && 
            filename.substr(filename.size() - 6) == ".route") {
            fileList.push_back(filename);
        }
    }
    
    closedir(dir);
    
    // Sort files by name
    std::sort(fileList.begin(), fileList.end());
    
    return fileList;
}

// Methods to support the task function
void RouteRecorder::addFrame(const SensorFrame& frame) {
    if (m_recording) {
        m_frames.push_back(frame);
    }
}

int RouteRecorder::getSampleRate() const {
    return m_sampleRateMs;
} 