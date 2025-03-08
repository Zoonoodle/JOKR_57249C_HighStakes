#ifndef SORTINGCONTROL_H
#define SORTINGCONTROL_H

#include <vector>

extern int ringDistance;
extern int colorCount;
extern bool intakeTaskRunning;

extern bool sortingTaskRunning;

enum intakeCommand {
    INTAKENONE,
    // SORTSECOND,
    // SORTTHIRD,
    STOPFIRST,
    STOPSECOND,
    STOPAFTER,
    STOPTHIRD,
    SORTFIRST,
    STOPRED,
    STOPBLUE,
    unclampAfterOne
};

extern intakeCommand currentIntakeCommand;
void intakeControlTask(void *param);

enum sortingCommand {
    SORTINGNONE,
    SORT_BLUE,
    SORT_RED
};

extern sortingCommand currentSortingCommand;
void sortingControlTask(void *param);
#endif 
