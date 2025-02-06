#ifndef MACROS_H
#define MACROS_H

// arm macros
void resetStates();
void cycleLoadState();
void cycleDescoreState();
void cycleTipState();
void ArmStateControl();
void setAutonState(int state);
void preLoadControl();
void secondLoadControl();
void secondScoreControl();
void liftWsAuto();
void AutopreLoadControl();
//intake macros
extern int currentArmState;
extern int currentAutonState;
extern int LoadStates[];
extern int targetArmState;
extern bool loadActivated;
void color_sort();
void eject_ring();
#endif