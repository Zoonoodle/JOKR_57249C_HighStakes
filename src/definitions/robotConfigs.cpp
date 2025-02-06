#include "main.h"
#include "robotConfigs.h"


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut mogo('G');
pros::ADIDigitalOut doinker('H');
pros::ADIDigitalOut intakeLift('A');
pros::ADIDigitalOut pincher('C');
pros::ADIDigitalIn limit('F');




pros::Motor intake(1, pros::MotorGearset::blue);

pros::Rotation armSensor(20);
pros::MotorGroup left_motors({-16, -18, -8}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, 2, 12}, pros::MotorGearset::blue);
pros::MotorGroup armMotors({11, -13}, pros::MotorGearset::green);

pros::Rotation hooks_rot(9);
lemlib::ControllerSettings latControllerNoMogo(9, 0.0001, 6.8, 5, 1, 100,
 4, 500, 0);
 lemlib::ControllerSettings latOmni(11, 0.01, 2, 10, 0.5, 100, 4, 500, 0);
lemlib::ControllerSettings angOmni(1.8, 0, 10.0, 3.0, 0.5, 100.0, 3.0, 500.0, 10);

 lemlib::ControllerSettings latControllerMogo(9, 0.0001, 16, 5, 1, 100,
 4, 500, 0);

  lemlib::ControllerSettings latController2Rings(8.2, 0, 18, 5, 1, 100,
 4, 500, 0);


lemlib::ControllerSettings angController(2.35, 0, 14, 10, 0.5, 100.0, 3.0, 500.0, 0);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);


pros::Imu imu(21);
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11, lemlib::Omniwheel::NEW_275, 450, 8); //8 for traction 2 for omnis


lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, latControllerNoMogo, angController, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis mogoChassis(drivetrain, latControllerMogo, angController, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis twoRingChassis(drivetrain, latController2Rings, angController, sensors, &throttle_curve, &steer_curve);
