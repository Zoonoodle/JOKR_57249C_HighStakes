#ifndef HARDWAREANDSENSORS_H
#define HARDWAREANDSENSORS_H

#include "pros/adi.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

// Declare all global objects here using extern

extern pros::Controller controller;  // Declare controller
extern lemlib::Chassis chassis;      // Declare chassis
extern pros::ADIDigitalOut mogo;     // Declare mogo mechanism (pneumatic)
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut intakeLift;
extern pros::ADIDigitalOut pincher;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Optical sorter;
extern pros::Rotation hooks_rot;
extern pros::MotorGroup armMotors; // Declare intake motors
extern pros::Motor intake;
extern bool intakeOverride;
extern bool loadActivated;
extern pros::ADIDigitalOut hang;
extern pros::Rotation armSensor;
extern pros::Distance distSorter;
extern double armZeroOffset;
extern bool armIsMoving;
extern bool autowsLoad;
// Initialize sensor objects
extern pros::Imu imu1;
extern pros::Imu imu2;
extern pros::Imu imu3;
extern pros::Optical sorter;
extern pros::Distance frontDistLeft;
extern pros::Distance rightDist;
extern pros::Distance frontDistRight;
extern pros::Distance leftDist;

extern lemlib::OdomSensors sensors;

extern pros::Controller master;

extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut intakeLift;
extern pros::ADIDigitalOut pincher;
extern pros::ADIDigitalIn limit;



extern bool mogoActivated;
extern bool intakeActivated;
extern bool doinkerActivated;
extern bool doinker2Activated;
extern pros::Motor intake;
extern pros::Rotation armSensor;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup armMotors;

extern pros::Rotation hooks_rot;
extern pros::Rotation vertical_encoder;
extern pros::Rotation horizontal_encoder;

extern lemlib::TrackingWheel vertical;
extern lemlib::TrackingWheel horizontal;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings latNoMogo;
extern lemlib::ControllerSettings angNoMogo;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

// extern static MultiIMU multi;
// extern static MultiIMUWrapper fusedIMU;
extern pros::Imu imu;

extern lemlib::OdomSensors sensors;
extern lemlib::Chassis chassis;



#endif
