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
#define SEMAPHORE_TOPIC "/semaphore"
#define SEMAPHORE_STATE_TOPIC "/conde_signalling_panel_state"
#define PARK_TOPIC "/park"

#define START_WAYPOINT 1

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

string getSemaphoreName(SemaphoreState s)
{
    switch (s)
    {
    case LEFT:
        return "LEFT";
    case RIGHT:
        return "RIGHT";
    case UP:
        return "UP";
    case STOP:
        return "STOP";
    case PARK:
        return "PARK";
    }
}

string getSensorName(Sensor s)
{
    switch (s)
    {
    case WAYPOINT:
        return "WAYPOINT";
    case TRACK:
        return "TRACK";
    case TRACK_BOUNDS:
        return "TRACK_BOUNDS";
    case TRACK_INSIDE:
        return "TRACK_INSIDE";
    case TRACK_OUTSIDE:
        return "TRACK_OUTSIDE";
    case SEMAPHORE:
        return "SEMAPHORE";
    case PARKING:
        return "PARKING";
    }
}

Sensor getSensor(string name)
{
    if (name.compare("WAYPOINT") == 0)
        return WAYPOINT;
    else if (name.compare("TRACK") == 0)
        return TRACK;
    else if (name.compare("TRACK_BOUNDS") == 0)
        return TRACK_BOUNDS;
    else if (name.compare("TRACK_INSIDE") == 0)
        return TRACK_INSIDE;
    else if (name.compare("TRACK_OUTSIDE") == 0)
        return TRACK_OUTSIDE;
    else if (name.compare("SEMAPHORE") == 0)
        return SEMAPHORE;
    else if (name.compare("PARKING") == 0)
        return PARKING;
}

string buildMessage(Sensor s, string collision1, string collision2)
{
    vector<string> parts1 = vector<string>();
    vector<string> parts2 = vector<string>();
    vector<string> parts3 = vector<string>();

    split_regex(parts1, collision1, regex("::"));
    split_regex(parts2, collision2, regex("::"));

    string robotName = "";
    string sensorName = "";

    if (parts1[1].compare("bumper_sensor") == 0)
    {
        sensorName = parts1[0];
        robotName = parts2[0];
    }
    else
    {
        robotName = parts1[0];
        sensorName = parts2[0];
    }

    string message = "";

    switch (s)
    {
    case WAYPOINT:
    {
        split_regex(parts3, sensorName, regex("_"));
        message = getSensorName(s) + SEPARATOR + robotName + SEPARATOR + parts3[parts3.size() - 1];
        break;
    }
    case TRACK:
    {
        string sensorType = to_upper_copy<string>(sensorName);
        message = getSensorName(s) + SEPARATOR + sensorType + SEPARATOR + robotName + SEPARATOR + parts1[1];
        cout << message << endl;
        break;
    }
    case SEMAPHORE:
    case PARKING:
    {
        message = getSensorName(s) + SEPARATOR + robotName;
        break;
    }
    }

    return message;
}

#endif