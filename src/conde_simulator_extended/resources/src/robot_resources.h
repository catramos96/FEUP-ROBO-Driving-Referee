#ifndef _ROBOT_RESOURCES_H_
#define _ROBOT_RESOURCES_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <chrono>
#include <ctime>

#include "utils.h"
#include "logic.h"

using namespace std;
using namespace std::chrono;

class Robot
{
  public:
    string name = "";
    high_resolution_clock::time_point start_time;
    high_resolution_clock::time_point end_time;
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
    void startRace() { start_time = high_resolution_clock::now(); };
    void endRace() { end_time = high_resolution_clock::now(); };
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
    double getRaceTime()
    {
        if (!hasFinishRace(route))
            return -1;
        else
            return duration_cast<chrono::milliseconds>(end_time - start_time).count();
    }
    bool consumeRouteWaypoint(int waypoint, SemaphoreState state)
    {
        if (!hasFinishRace(route))
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
                    if (route.size() == 0)
                        startRace();

                    if (state == STOP)
                    {
                        cout << "TODO: Apply Penalization and logic" << endl;
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

                //finish route
                if (hasFinishRace(route))
                {
                    endRace();
                    next_route = {};
                }

                print();
            }
            else if (waypoint != getLastWaypoint())
            {
                cout << "WRONG WAYPOINT: Waypoint should have been " << next_waypoint << " but was " << waypoint << endl;
            }
        }
    };
    void print(void)
    {
        string p = "NAME: " + name + "\n";
        p += "LAPS: " + to_string(getCurrentLap(route)) + "/" + to_string(LAPS) + "\n";
        if (hasFinishRace(route))
        {
            p += "STATUS: finished\n";
            auto m = duration_cast<chrono::minutes>(end_time - start_time).count();
            auto s = duration_cast<chrono::seconds>(end_time - start_time).count() - m * 60;
            auto ms = duration_cast<chrono::milliseconds>(end_time - start_time).count() - s * 1000;
            p += "ELAPSED TIME: " + to_string(m) + ":" + to_string(s) + "." + to_string(ms) + "\n";
        }
        else
        {
            p += "STATUS: ongoing\n";
            high_resolution_clock::time_point current = high_resolution_clock::now();
            auto m = duration_cast<chrono::minutes>(current - start_time).count();
            auto s = duration_cast<chrono::seconds>(current - start_time).count() - m * 60;
            auto ms = duration_cast<chrono::milliseconds>(current - start_time).count() - s * 1000;
            p += "ELAPSED TIME: " + to_string(m) + ":" + to_string(s) + "." + to_string(ms) + "\n";
        }

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