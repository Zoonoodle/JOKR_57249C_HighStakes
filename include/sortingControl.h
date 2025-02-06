#ifndef SORTINGCONTROL_H
#define SORTINGCONTROL_H

#include <vector>

extern int ringDistance;
extern int colorCount;
extern bool sortingTaskRunning;


enum sortingCommand {
    SORTINGNONE,
    // SORTSECOND,
    // SORTTHIRD,
    STOPFIRST,
    STOPSECOND,
    STOPAFTER,
    STOPTHIRD,
    // SORTFIRST
};

extern sortingCommand currentSortingCommand;

void sortingControlTask(void *param);

#endif 
