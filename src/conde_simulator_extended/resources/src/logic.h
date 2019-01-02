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
    vector<int> route = vector<int>();

    switch (semaphore)
    {
    case UP:
    {
        if (to_right)
            route = {2, 3, 4, 5, 1};
        else
            route = {6, 7, 8, 1};
        break;
    }
    case STOP:
    {
        route = {1};
        break;
    }
    case LEFT:
    {
        if (to_right)
            route = {5, 4, 3, 2, 1};
        else
            route = {6, 7, 8, 1}; //ignore and go up
        break;
    }
    case RIGHT:
    {
        if (to_right)
            route = {2, 3, 4, 5, 1}; //ignore and go up
        else
            route = {8, 7, 6, 1};
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

#endif