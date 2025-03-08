#include "bennyHeaders/hardwareAndSensors.h"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/rotation.hpp"
#include "robotConfigs.h"


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut mogo('H');
pros::ADIDigitalOut doinker('G');
pros::ADIDigitalOut doinker2('A');
pros::ADIDigitalOut intakeLift('E');





pros::Motor intake(12, pros::MotorGearset::blue);

pros::Rotation armSensor(20);
pros::MotorGroup left_motors({-13, -18, -7}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, 8, 4}, pros::MotorGearset::blue);
pros::MotorGroup armMotors({16, -11}, pros::MotorGearset::green);

pros::Rotation hooks_rot(-3); //empty

pros::Rotation horizontal_encoder(9);
pros::Rotation vertical_encoder(10);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontal_encoder, lemlib::Omniwheel::NEW_275, -2.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);





lemlib::ControllerSettings latControllerNoMogo(5.75  ,
                                         0, 0.6,5, 1, 100,
 4, 500, 0);
 lemlib::ControllerSettings latOmni(11, 0.01, 2, 10, 0.5, 100, 4, 500, 0);
lemlib::ControllerSettings angOmni(1.8, 0, 10.0, 3.0, 0.5, 100.0, 3.0, 500.0, 10);

 lemlib::ControllerSettings latControllerMogo(6, 0.0001, 1.1, 5, 1, 100,
 4, 500, 0);

  lemlib::ControllerSettings latController2Rings(5.9, 0.0001, 1.1, 5, 1, 100,
    4, 500, 0);



    lemlib::ControllerSettings tractionLateral(7.0, 
      0,
       14 ,
        5,
        0.5,
        100,
      5,
        500,
        0);
    

                                            
    lemlib::ControllerSettings tractionAngular(2.71, 
                                        0,
                                          18.2,
                                          3,
                                          0.5,
                                          100,
                                          3,
                                          500,
                                          0);

lemlib::ControllerSettings angController(2.5, 0, 16.2, 0, 0.5, 100.0, 3.0, 500.0, 0);

lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);


pros::Imu imu(21);
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11, lemlib::Omniwheel::NEW_275, 450, 8); //8 for traction 2 for omnis


lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

bool isOmni = true;

  
lemlib::Chassis chassis(drivetrain, tractionLateral, tractionAngular, sensors, &throttle_curve, &steer_curve); 


lemlib::Chassis mogoChassis(drivetrain, tractionLateral, tractionAngular, sensors, &throttle_curve, &steer_curve);
lemlib::Chassis twoRingChassis(drivetrain, tractionLateral, angController, sensors, &throttle_curve, &steer_curve);
bool autonDisabled = false;