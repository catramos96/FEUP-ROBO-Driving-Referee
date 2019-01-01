#ifndef _UTILS_HH_
#define _UTILS_HH_

#define BOUNDARIES_TOPIC "/boundaries_colisions"
#define ROBOT_TIME_TOPIC "/conde_referee_robot_time"
#define WAYPOINTS_TOPIC "/waypoints"
#define SEMAPHORE_TOPIC "/semaphore"
#define SEMAPHORE_STATE_TOPIC "/conde_signalling_panel_state"

#define DEBUG false

enum SemaphoreState
{
    LEFT,
    RIGHT,
    UP,
    STOP,
    PARK
};

#endif