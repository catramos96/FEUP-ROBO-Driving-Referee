#ifndef _ROBOT_RESOURCES_H_
#define _ROBOT_RESOURCES_H_

#include <string>
#include <vector>
#include <iostream>
#include "utils.h"
#include <sstream>
#include <time.h>

using namespace std;

class Robot
{
  public:
    string name = "";
    time_t start_time = -1;
    time_t end_time = -1;
    vector<int> route = vector<int>();
    int score = 0;
    int penalties = 0;
    SemaphoreState last_semaphore = UP;

  public:
    Robot(string name) { setName(name); };
    void setName(string new_name) { name = new_name; };
    void start_race() { start_time = time(0); };
    void end_race() { end_time = time(0); };
    void addWaypoint(int waypoint) { route.push_back(waypoint); };
    void addPenalty(int new_penalty) { penalties += new_penalty; };
    void addScore(int new_score) { score += new_score; };
    void updateSemState(SemaphoreState state) { last_semaphore = state; };
    int getLastWaypoint() { return route[route.size() - 1]; };
    void print(void)
    {
        char buffer1[100];
        char buffer2[100];
        struct tm *tm_info1, *tm_info2;

        strftime(buffer1, 100, "%Y-%m-%d %H:%M:%S", localtime(&start_time));
        strftime(buffer2, 100, "%Y-%m-%d %H:%M:%S", localtime(&end_time));

        string st = buffer1;
        string et = buffer2;

        string p = "NAME: " + name + "\n";
        p += "START TIME: " + st + " , RAW: " + to_string(start_time) + "\n";
        //p += "END TIME: " + et + " , RAW: " + to_string(end_time) + "\n";
        p += "ROUTE: ";

        for (int i = 0; i < route.size(); i++)
        {
            if (i == 0)
                p += to_string(route[i]);
            else
                p += " - " + to_string(route[i]);
        }

        p += "\nSCORE: " + to_string(score);
        p += "\nPENALTIES: " + to_string(penalties);
        p += "\nSEMAPHORE: " + getSemaphoreName(last_semaphore) + "\n\n";

        cout << p;
    };
};

#endif