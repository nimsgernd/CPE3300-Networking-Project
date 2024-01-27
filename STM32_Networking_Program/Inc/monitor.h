#ifndef MONITOR_H_
#define MONITOR_H_

void monitor_init(void);
void monitor(void);

enum State 
{
    BUSY,
    IDLE,
    COLLISION
};


#endif
