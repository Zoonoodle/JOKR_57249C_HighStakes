#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/macros.h"


bool descoreActivated = false;
bool loadActivated    = false;
bool tipActivated     = false;
bool intakeOverride   = false;

bool secondLoaded     = false;
bool fromLoad         = false;
bool firstLoad = false;
bool fromPreScore = false;
bool autowsLoad = false;




const int numOfLoadStates    = 3; 
const int numOfDescoreStates = 2;
const int numOfTipStates     = 2;
const int numOfAutonStates   = 6;


int neutralPos     = 0;
int loadPos        = 12000; 
int preScorePos    = 18000; 
int scorePos       = 53000; 
int descoreWallPos = 50000;
int tippingPos     = 62000;
int untippingPos   = 74000;
int alliancePos    = 65000;


int LoadStates[numOfLoadStates]       = { loadPos,      preScorePos,    scorePos    };
int DescoreStates[numOfDescoreStates] = { neutralPos,   descoreWallPos };
int tipStates[numOfTipStates]         = { tippingPos,   untippingPos   };
int autonStates[numOfAutonStates]     = { neutralPos,   loadPos,        59000, alliancePos, 73000, preScorePos};





int currentArmState     = -1;  
int targetArmState      = 0;
int currentTipState     = -1;
int currentAutonState   = 0;
int currentDescoreState = -1;




void resetStates() {
  
  descoreActivated   = false;
  loadActivated      = false;
  tipActivated       = false;
  secondLoaded       = false;
  fromLoad           = false;
  intakeOverride     = false;
  secondLoaded = false;
  
  currentArmState    = -1;  
  currentTipState    = -1;
  currentDescoreState= -1;

  
  targetArmState   = neutralPos;
}




void cycleTipState() {
  
  currentDescoreState = -1;
  currentArmState     = -1;
  loadActivated       = false;
  descoreActivated    = false;

  tipActivated        = true;
  currentTipState++;
  if (currentTipState == numOfTipStates) {
    currentTipState = 0;
  }

  targetArmState = tipStates[currentTipState];
}




void cycleDescoreState() {
  resetStates(); 

  descoreActivated   = true;
  currentDescoreState++;
  if (currentDescoreState == numOfDescoreStates) {
    currentDescoreState = 0;
  }
  targetArmState = DescoreStates[currentDescoreState];
}





void cycleLoadState() {
  currentDescoreState = -1;
  currentTipState     = -1;
  descoreActivated    = false;
  tipActivated        = false;

  loadActivated = true;
  currentArmState++;

  if (currentArmState >= numOfLoadStates) {
    currentArmState = 0; 
    fromLoad = true;
    firstLoad = false;
  }
  if (currentArmState == 1) {
    currentArmState = 2;
  }

  

  targetArmState = LoadStates[currentArmState];
}




void setAutonState(int autoState) {
  resetStates();
  currentAutonState = autoState;
  targetArmState    = autonStates[currentAutonState];
}







void preLoadControl() {
  while (true) {
    
    if (currentArmState == 1 && !secondLoaded) {
      
      intakeOverride = true;
      hooks_rot.reset_position();
      intake.move(-100);
      
      while (hooks_rot.get_position() > -1000) {
        pros::delay(10);
      }

      
      pros::delay(200);
      fromPreScore = true;
      firstLoad = true;
      intake.move(127);

      
      while (sorter.get_proximity() < 100) {
        pros::delay(10);
      }
      hooks_rot.reset_position();
      while (hooks_rot.get_position() < 17000) {
        pros::delay(10);
      }
      
      
      intake.move(0);

      
      if (firstLoad) {
        secondLoaded = true;
      }
      

      

      
    }
    pros::delay(10);
  }
}

void AutopreLoadControl() {
while (true) {


    
    if (currentAutonState == 5) {
      
      intakeOverride = true;
      hooks_rot.reset_position();
      intake.move(0);
      
     

      
      pros::delay(200);
      fromPreScore = true;
      firstLoad = true;
      intake.move(127);

      
      while (sorter.get_proximity() < 100) {
        pros::delay(10);
      }
      hooks_rot.reset_position();
      while (hooks_rot.get_position() < 26000) {
        pros::delay(10);
      }
      
      
      intake.move(0);
      

      

      
    }

  pros::delay(10);
}

}


void liftWsAuto() {
    while(true) {

    if (currentAutonState == 1) {

    
    while (sorter.get_proximity() < 100) {
        pros::delay(10);
      }
      hooks_rot.reset_position();
      while (hooks_rot.get_position() < 26000) {
        pros::delay(10);
      }
      // pros::delay(200);
      intake.move(0);
      currentAutonState = 5;
      targetArmState = LoadStates[currentAutonState];
    autowsLoad = false;
}
    }
pros::delay(10);
}


void secondScoreControl() {
  while (true) {
    if (currentArmState == 0 && secondLoaded) {

      while (armSensor.get_position() > loadPos + 100) {
        pros::delay(10);
      }
      
      pros::delay(250);
      intakeOverride = true;
      intake.move(127);
      pros::delay(200); 
      intake.move(0);
      
    
      
      currentArmState = 2;
      fromPreScore = true;
      targetArmState  = LoadStates[2] + 2000; 

      
      
      pros::delay(750);

  
      intakeOverride = false;
    

   

      
      secondLoaded = false;

     
      resetStates();
      // currentArmState = 0;
      // targetArmState = LoadStates[currentArmState];
    }
    pros::delay(10);
  }
}




void ArmStateControl() {
  double kp = 0.007; 
  double kd = 0.001;
  static double prevError = 0;

  while (true) {
    
    if (fromPreScore || loadActivated && currentArmState == 2) {
      kp = 0.012; 
      kd = 0.001; 
      fromPreScore = false;
    } 
    else if (fromLoad && currentArmState == 0) {
      kp = 0.009;
      kd = 0.00025;
      fromLoad = false;
    }
    else if (currentAutonState == 4) {
      kp = 0.012;
      kd = 0.001;
    }
    else if (currentAutonState == 1) {
      kp = 0.004;
      kd = 0.0005;
    }
    else if (currentArmState == 1) {
      kp = 0.003;
      kd = 0.0005;
    } 
    else {
      
      kp = 0.006;
      kd = 0.0003;
    }

    double error = targetArmState - armSensor.get_position();
    double deriv = error - prevError;
    prevError = error;

    double velocity = (kp * error) + (kd * deriv);
    armMotors.move(velocity);

    pros::delay(10);
  }
}
