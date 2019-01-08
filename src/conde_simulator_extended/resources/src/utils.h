#ifndef _UTILS_H_
#define _UTILS_H_

#include <string>
#include <vector>
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <ctime>

#define BOUNDARIES_TOPIC "/boundaries_colisions"
#define WAYPOINTS_TOPIC "/waypoints"
#define SEMAPHORE_STATE_TOPIC "/conde_signalling_panel_state"
#define PARK_TOPIC "/park"

#define START_WAYPOINT 1
#define LAPS 2
#define PARKING_TIME 5 //seconds
#define BOUNDS_PENALTY_TIME_INTERVAL 2
#define SEMAPHORE_PENALTY_TIME_INTERVAL 2
#define PARKING_PENALTY_TIME_INTERVAL 2
#define DRIVING_SCORE_REFERENCE 50
#define PARKING_SCORE_REFERENCE 60
#define TIME_REFERENCE 35

#define SEPARATOR "##"

#define DEBUG false

using namespace std;
using namespace boost;

enum Sensor
{
    WAYPOINT,
    SEMAPHORE,
    TRACK,
    TRACK_BOUNDS,
    TRACK_INSIDE,
    TRACK_OUTSIDE,
    PARKING
};

enum SemaphoreState
{
    LEFT,
    RIGHT,
    UP,
    STOP,
    PARK
};

enum RaceState
{
    WAITING,
    ONGOING,
    BAY_PARKING,
    PARALLEL_PARKING,
    FINISHED,
    DISQUALIFIED
};

string getSemaphoreName(SemaphoreState s);
string getSensorName(Sensor s);
string getRaceStateName(RaceState s);
Sensor getSensor(string name);

string buildMessage(Sensor s, string collision1, string collision2);

#endif