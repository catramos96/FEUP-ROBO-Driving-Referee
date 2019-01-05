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
*/
vector<int> getNextRoute(SemaphoreState semaphore, bool to_right)
{
    vector<int> route;

    switch (semaphore)
    {
    case UP:
    {
        if (to_right) {
            int aux[] = {2, 3, 4, 5, 1};
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        } else {
            int aux[] = {6, 7, 8, 1};
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        }            
        break;
    }
    case STOP:
    {
        int aux[] = {1};
        route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        break;
    }
    case LEFT:
    {
        if (to_right) {
            int aux[] = {5, 4, 3, 2, 1};
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        } else {
            int aux[] = {6, 7, 8, 1}; //ignore and go up
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        }
        break;
    }
    case RIGHT:
    {
        if (to_right) {
            int aux[] = {2, 3, 4, 5, 1}; //ignore and go up
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        } else {
            int aux[] = {8, 7, 6, 1};
            route.insert(route.end(), aux, aux+(sizeof(aux)/sizeof(aux[0])));
        }            
        break;
    }
    case PARK:
    {
        //TODO
        break;
    }
    }

    return route;
}

bool goesToRight(int last_waypoint)
{
    if (last_waypoint > 5)
        return true;
    else
        return false;
}

int nWaypointsLeft(vector<int> route)
{
    return (9 * LAPS + 1) - route.size();
}

bool hasFinishRace(vector<int> route)
{

    if (nWaypointsLeft(route) == 0)
        return true;
    else
        return false;
}

int getCurrentLap(vector<int> route)
{
    return floor(route.size() / (9 * LAPS + 1));
}

#endif