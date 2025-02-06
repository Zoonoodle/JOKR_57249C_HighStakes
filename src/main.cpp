#include "main.h"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/macros.h"
#include "bennyHeaders/autons.h"
#include "bennyHeaders/DistanceSensorMovements.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robotConfigs.h"
#include "sortingControl.h"





void initialize() {
  std::int32_t set_integration_time(20);
    pros::lcd::initialize(); 
    armSensor.reset_position();
    
    chassis.calibrate(); 
    
    hooks_rot.reset_position();

    pros::Task  preLoadControlTask([] {
      while(true) {
      preLoadControl();
      pros::delay(10);
      }
      
    });

    
    
    
    
    

    pros::Task autoWsTask([] {
        while (true) {
          AutopreLoadControl();
        }
      
    });


    // pros::Task autoLoadTask([] {
    //   while(true) {
    //     liftWsAuto();
    //   }
    // });
    pros::Task secondScoreTask([] {
      while (true) {
        secondScoreControl();
        pros::delay(10);
      }
    });
    pros::Task LoadControlTask([] {
      while (true) {
        ArmStateControl();
        pros::delay(10);
      }

      
    });
// pros::Task mancoltask([&]()  {
//   while (true) {
//     manCol();
//   }
// });
// pros::Task pidInit([] {
// while (true) {
//   lemlib::Chassis chassis(drivetrain, latNoMogo, angNoMogo, sensors, &throttle_curve, &steer_curve);
// }
// });

pros::Task screen_task([&]() {
        while (true) {
            
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); 
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); 
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); 
            

            pros::lcd::print(0, "left_temps: %f", left_motors.get_temperature_all());
            pros::lcd::print(1, "right_temps: %f", right_motors.get_temperature_all());  
            pros::delay(20);
        }
    });
}



void disabled() {
  mogo.set_value(true);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


void competition_initialize() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


void autonomous() {
chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);


     int autonSelector = 0;  

    switch (autonSelector) {
        case 0:
        bristolSkills();
        break;

        case 1:
        
            break;
        case 2:
            redSideMogoRush();
            break;
        case 3:
         
            blueSideMogoRush();
            break;
        case 4:
  
            break;
        case 5:
            blueSawp();
            break;
        case 6:
        redSawp();
            break;
            case 7:

            redSawp2();
            break;
    }
    


    
}












bool mogoActivated = false;
bool intakeActivated = false;
bool doinkerActivated = false;







void opcontrol() {


  
  setAutonState(0);
  setAutonState(1);
    pros::delay(250);
        intake.move(127);
    pros::delay(400);
    intake.move(0);
    pros::delay(100);
    setAutonState(3);
    pros::delay(500);
    setAutonState(0);
    resetStates();
  




  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    std::cout << "Motor Temperature: " << left_motors.get_temperature_all()[1];
    int leftY  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX);

    
    if (!intakeOverride) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move(127);
      }
      else {
        intake.move(0);
      }
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      cycleLoadState();
    }

    
    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      
      if (currentArmState == 0) {
        
        currentArmState = 1; 
        loadActivated   = true; 
        targetArmState  = LoadStates[1]; 
      } 
      else {
        
        cycleTipState();
      }
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      resetStates();
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      doinkerActivated = !doinkerActivated;
      doinker.set_value(doinkerActivated);
    }

    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //      cycleTipState();
    // }
    
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    //   sortOn = true;
    intake.move(-127);
    }

    
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      mogoActivated = !mogoActivated;
      mogo.set_value(mogoActivated);
    }

    pros::delay(20);
  }
}








