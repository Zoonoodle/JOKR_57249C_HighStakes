#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "lemlib/api.hpp"

// Controllers
extern pros::Controller master;

// Digital Out
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut doinker;
extern pros::ADIDigitalOut doinker2;
extern pros::ADIDigitalOut intakeLift;
extern pros::ADIDigitalOut pincher;
extern bool autonDisabled;
// Digital In
extern pros::ADIDigitalIn limit;



// Motors
extern pros::Motor intake;
extern pros::Rotation armSensor;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup armMotors;
extern pros::Rotation hooks_rot;

// Controller Settings
extern lemlib::ControllerSettings latControllerNoMogo;
extern lemlib::ControllerSettings latOmni;
extern lemlib::ControllerSettings angOmni;
extern lemlib::ControllerSettings latControllerMogo;
extern lemlib::ControllerSettings latController2Rings;
extern lemlib::ControllerSettings angController;

// Drive Curve
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

// IMU
extern pros::Imu imu;

// Drivetrain
extern lemlib::Drivetrain drivetrain;

// Odom Sensors
extern lemlib::OdomSensors sensors;

// Chassis
extern lemlib::Chassis chassis;
extern lemlib::Chassis mogoChassis;
extern lemlib::Chassis twoRingChassis;

#endif // _ROBOT_CONFIG_H_
