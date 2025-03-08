#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "bennyHeaders/hardwareAndSensors.h"
#include "bennyHeaders/macros.h"
#include "robotConfigs.h"

bool descoreActivated = false;
bool loadActivated    = false;
bool tipActivated     = false;
bool intakeOverride   = false;

bool secondLoaded     = false;
bool fromLoad         = false;
bool firstLoad = false;
bool fromPreScore = false;
bool autowsLoad = false;
bool isAlliance= false;



const int numOfLoadStates    = 3; 
const int numOfDescoreStates = 2;
const int numOfTipStates     = 2;
const int numOfAutonStates   = 8;
const int numOfAllianceStates = 2;

int neutralPos     = 0;
int loadPos        = 11500; 
int preScorePos    = 17000; 
int scorePos       = 52000; 
int descoreWallPos = 48000;
int preWSState = 41000;
int tippingPos     = 60000;
int untippingPos   = 74000;
int alliancePos    = 61500;
int preAlliancePos = 49000;

int LoadStates[numOfLoadStates]       = { loadPos,      preScorePos,    scorePos};
int DescoreStates[numOfDescoreStates] = { neutralPos,   descoreWallPos };
int tipStates[numOfTipStates]         = { tippingPos,   untippingPos   };
int autonStates[numOfAutonStates]     = { neutralPos,   loadPos,        59000, alliancePos, 73000, preScorePos, preAlliancePos, preWSState};
int allianceStates[numOfAllianceStates] = {preAlliancePos, alliancePos};




int currentArmState     = -1;  
int targetArmState      = 0;
int currentTipState     = -1;
int currentAutonState   = 0;
int currentDescoreState = -1;
int currentAllianceState = -1;



void resetStates() {
  
  descoreActivated   = false;
  loadActivated      = false;
  tipActivated       = false;
  secondLoaded       = false;
  fromLoad           = false;
  intakeOverride     = false;
  secondLoaded = false;
  
  currentAllianceState = -1;
  currentArmState    = -1;  
  currentTipState    = -1;


  
  // targetArmState   = neutralPos;
}

void resetArmPos() {
  targetArmState = -11500;
  pros::delay(100);
  armSensor.reset_position();
  targetArmState = 0;
  resetStates();
}


void cycleTipState() {
  
  currentDescoreState = -1;
  currentArmState     = -1;
  loadActivated       = false;
  descoreActivated    = false;
  //  secondLoaded = false; 
  tipActivated        = true;
  currentTipState++;
  if (currentTipState == numOfTipStates) {
    currentTipState = 0;
  }

  targetArmState = tipStates[currentTipState];
}




void cycleDescoreState() {
  
  
    descoreActivated   = true;
    currentDescoreState++;
    if (currentDescoreState == numOfDescoreStates) {
      currentDescoreState = 0;
    }
    targetArmState = DescoreStates[currentDescoreState];
 
    resetStates(); 
  
 
    
}






void cycleLoadState() {
  currentDescoreState = -1;
  currentTipState     = -1;
  descoreActivated    = false;
  tipActivated        = false;
  currentAllianceState = -1;
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

  if (currentAutonState == 6) {
    isAlliance = true;
  }
  targetArmState    = autonStates[currentAutonState];
}

void allianceStake() {

  currentDescoreState = -1;
  currentTipState     = -1;
  descoreActivated    = false;
  tipActivated        = false;

 
  currentAllianceState++;

  if (currentAllianceState >= numOfAllianceStates) {
    currentAllianceState = 0; 
   
  }


  
isAlliance = true;
  targetArmState = allianceStates[currentAllianceState];
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

      
      while (sorter.get_proximity() < 150) {
        pros::delay(10);
      }
    pros::delay(20);
      
      
      intake.move(0);

      
      if (firstLoad) {
        secondLoaded = true;
      }
      if (autonDisabled) {
        pros::delay(300);
        targetArmState = LoadStates[0];
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
      

    


      pros::delay(20);
      intakeOverride = false;
    //     intakeOverride = false;
    //   intakeOverride = true;
    //   intake.move(127);
    //   pros::delay(200); 
    //   intake.move(0);
      
    
      
    //   currentArmState = 2;
    //   fromPreScore = true;
    //   targetArmState  = LoadStates[2] + 2000; 

      
      
    

  
    //   intakeOverride = false;
    

   

      
    //   secondLoaded = false;

     

    //   // currentArmState = 0;
    //   // targetArmState = LoadStates[currentArmState];
     }
    pros::delay(10);
  }
}




void ArmStateControl() {
    double kp = 0.004; 
  double kd = 0.004;
  static double prevError = 0;

  while (true) {
    
    if (fromPreScore || loadActivated && currentArmState == 2) {
      kp = 0.013; 
      kd = 0.0009; 
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

    if (descoreActivated) {
      descoreActivated = false;
      double kp = 0.003; 
      double kd = 0.005;
    }

    if (isAlliance) {
      kp = 0.003;
      kd = 0.0008;
      isAlliance = false;
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
