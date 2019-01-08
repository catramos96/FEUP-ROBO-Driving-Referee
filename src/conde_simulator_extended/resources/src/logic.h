#ifndef _LOGIC_H_
#define _LOGIC_H_

#include <string>
#include <vector>
#include <iostream>
#include "utils.h"

using namespace std;

/*
 ---8--     ----4----
 -    -     -       -
 7    --1---5	    3
 -      1           -
 ---6---1-------2----
 ---9----------10---- 
*/
vector<int> getNextRoute(SemaphoreState semaphore, bool to_right);

bool goesToRight(int last_waypoint);

int nWaypointsLeft(vector<int> route);

bool hasFinishRace(vector<int> route);

int getCurrentLap(vector<int> route);

bool isParkingWaypoint(int waypoint);

#endif