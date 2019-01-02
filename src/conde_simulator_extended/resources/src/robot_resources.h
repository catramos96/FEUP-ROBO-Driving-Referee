#ifndef _ROBOT_RESOURCES_H_
#define _ROBOT_RESOURCES_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <time.h>

#include "utils.h"
#include "logic.h"

using namespace std;

class Robot
{
  public:
    string name = "";
    time_t start_time = -1;
    time_t end_time = -1;
    vector<int> route = vector<int>();
    vector<int> next_route = vector<int>(); //route to be performed
    int score = 0;
    int penalties = 0;
    SemaphoreState last_semaphore = UP;

  public:
    Robot(string name)
    {
        setName(name);
        route = vector<int>();
        next_route = vector<int>();
        next_route.push_back(START_WAYPOINT);
    };
    void setName(string new_name) { name = new_name; };
    void start_race() { start_time = time(0); };
    void end_race() { end_time = time(0); }; //tmp
    void addPenalty(int new_penalty) { penalties += new_penalty; };
    void addScore(int new_score) { score += new_score; };
    void updateSemState(SemaphoreState state) { last_semaphore = state; };
    int getLastWaypoint()
    {
        if (route.size() != 0)
            return route[route.size() - 1];
        else
            return 8;
    };
    bool consumeRouteWaypoint(int waypoint, SemaphoreState state)
    {
        int next_waypoint = START_WAYPOINT;

        if (next_route.size() != 0)
            next_waypoint = next_route[0];
        else
        {
            next_route = {START_WAYPOINT};
            next_waypoint = START_WAYPOINT;
        }

        //corect path
        if (waypoint == next_waypoint)
        {
            //near semaphore
            if (waypoint == START_WAYPOINT)
            {
                //check if the robot already started the race
                if (start_time == -1)
                    start_race();

                if (state == STOP)
                {
                    cout << "TODO: Apply Penalization" << endl;
                }
                else
                {
                    next_route = getNextRoute(state, goesToRight(getLastWaypoint()));
                }
            }

            //add waypoint to route
            route.push_back(next_waypoint);

            //remove waypoint from next route if not removed
            //when route was updated (near semaphore)
            if (next_route[0] == next_waypoint)
                next_route.erase(next_route.begin());

            print();
        }
        else if (waypoint != getLastWaypoint())
        {
            cout << "WRONG WAYPOINT: Waypoint should have been " << next_waypoint << " but was " << waypoint << endl;
        }
    };
    void print(void)
    {
        char buffer1[100];
        char buffer2[100];

        time_t current = time(0);

        strftime(buffer1, 100, "%Y-%m-%d %H:%M:%S", localtime(&start_time));
        strftime(buffer2, 100, "%Y-%m-%d %H:%M:%S", localtime(&current));

        string st = buffer1;
        string et = buffer2;

        string p = "NAME: " + name + "\n";
        p += "START TIME: " + st + " , RAW: " + to_string(start_time) + "\n";
        p += "CURRENT TIME: " + et + " , RAW: " + to_string(current) + "\n";
        p += "ROUTE: ";

        for (int i = 0; i < route.size(); i++)
        {
            if (i == 0)
                p += to_string(route[i]);
            else
                p += " - " + to_string(route[i]);
        }

        p += "\nNEXT ROUTE: ";

        for (int i = 0; i < next_route.size(); i++)
        {
            if (i == 0)
                p += to_string(next_route[i]);
            else
                p += " - " + to_string(next_route[i]);
        }

        p += "\nSCORE: " + to_string(score);
        p += "\nPENALTIES: " + to_string(penalties);
        p += "\nSEMAPHORE: " + getSemaphoreName(last_semaphore) + "\n\n";

        cout << p;
    };
};

#endif