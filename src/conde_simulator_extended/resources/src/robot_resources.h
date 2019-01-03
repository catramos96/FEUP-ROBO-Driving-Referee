#ifndef _ROBOT_RESOURCES_H_
#define _ROBOT_RESOURCES_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "boost/lexical_cast.hpp"

#include "utils.h"
#include "logic.h"

using namespace std;

class RobotCollision {
    public:
        string component;
        bool isColliding;

    RobotCollision(string component, bool colliding)
    {
        setComponent(component);
        setCollisionState(colliding);
    };

    void setComponent(string new_component) { component = new_component; };
    void setCollisionState(bool state) { isColliding = state; };
};

class Robot
{
  public:
    string name;
    double start_time;
    double end_time;
    vector<int> route;
    vector<int> next_route; //route to be performed
    vector<RobotCollision *> insideCollisions; // info on wheter the robot is inside the track
    vector<RobotCollision *> boundariesCollisions; // info on wheter the robot is touching the boundaries
    vector<RobotCollision *> outsideCollisions; // info on wheter the robot is outside the track
    bool hadBoundaryCollision;
    int score;
    int penalties;
    SemaphoreState last_semaphore;

  public:
    Robot(string name)
    {
        score = 0;
        penalties = 0;
        last_semaphore = UP;
        setName(name);
        next_route.push_back(START_WAYPOINT);

        setBoundaryCollision(false);
        initializeCollisionVectors();
    };
    void setName(string new_name) { name = new_name; };
    void startRace() { start_time = ros::Time::now().toSec(); };
    void endRace() { end_time = ros::Time::now().toSec(); };
    void addPenalty(int new_penalty) { penalties += new_penalty; };
    void addScore(int new_score) { score += new_score; };
    void updateSemState(SemaphoreState state) { last_semaphore = state; };
    void setBoundaryCollision(bool state) { hadBoundaryCollision = state; };
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
            return (ros::Duration(end_time) - ros::Duration(start_time)).toSec();
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
                next_route.clear();
                next_route.push_back(START_WAYPOINT);
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
                    next_route.clear();
                }

                print();
            }
            else if (waypoint != getLastWaypoint())
            {
                cout << "WRONG WAYPOINT: Waypoint should have been " << next_waypoint << " but was " << waypoint << endl;
            }
        }
    };
    void initializeCollisionVectors() { 
        RobotCollision *right_wheel = new RobotCollision("right_wheel_collision", false);
        RobotCollision *left_wheel = new RobotCollision("left_wheel_collision", false);
        RobotCollision *front_wheel = new RobotCollision("chassis_collision", false);

        insideCollisions.push_back(right_wheel);
        insideCollisions.push_back(left_wheel);
        insideCollisions.push_back(front_wheel);
        
        outsideCollisions.push_back(right_wheel);
        outsideCollisions.push_back(left_wheel);
        outsideCollisions.push_back(front_wheel);

        boundariesCollisions.push_back(right_wheel);
        boundariesCollisions.push_back(left_wheel);
        boundariesCollisions.push_back(front_wheel);
    };
    bool setCollisionStateBySensor(string component, Sensor sensor, bool state) {
        switch (sensor)
        {
        case TRACK_OUTSIDE:
            for (int i = 0; i < outsideCollisions.size(); i++)
            {
                if (component.compare(outsideCollisions[i]->component) == 0)
                {
                    outsideCollisions[i]->setCollisionState(state);
                    return true;
                }
            }
            break;
        case TRACK_INSIDE:
            for (int i = 0; i < insideCollisions.size(); i++)
            {
                if (component.compare(insideCollisions[i]->component) == 0)
                {
                    insideCollisions[i]->setCollisionState(state);
                    return true;
                }
            }
            break;
        case TRACK_BOUNDS:
            for (int i = 0; i < boundariesCollisions.size(); i++)
            {
                if (component.compare(boundariesCollisions[i]->component) == 0)
                {
                    boundariesCollisions[i]->setCollisionState(state);
                    return true;
                }
            }
            break;
        }

        return false;
    }
    bool isInsideTrack() {
        for (int i = 0; i < insideCollisions.size(); i++)
        {
            if (insideCollisions[i]->isColliding == false)
            {
                return false;
            }
        }

        return true;
    }
    bool isOutsideTrack() {
        for (int i = 0; i < outsideCollisions.size(); i++)
        {
            if (outsideCollisions[i]->isColliding == false)
            {
                return false;
            }
        }

        return true;
    }
    void print(void)
    {
        string p = "NAME: " + name + "\n";
        p += "LAPS: " + getCurrentLap(route) +  boost::lexical_cast<string>("/") + boost::lexical_cast<string>(LAPS) + "\n";
        if (hasFinishRace(route))
        {
            p += "STATUS: finished\n";
            double m = (ros::Duration(end_time) - ros::Duration(start_time)).toSec();
            double s = (ros::Duration(end_time) - ros::Duration(start_time)).toSec() - m * 60;
            double ms = (ros::Duration(end_time) - ros::Duration(start_time)).toSec()- s * 1000;
            p += "ELAPSED TIME: " + boost::lexical_cast<string>(m) + ":" + boost::lexical_cast<string>(s) + "." + 
                boost::lexical_cast<string>(ms) + "\n";
        }
        else
        {
            p += "STATUS: ongoing\n";
            double current = ros::Time::now().toSec();
            double m = (ros::Duration(current) - ros::Duration(start_time)).toSec();
            double s = (ros::Duration(current) - ros::Duration(start_time)).toSec() - m * 60;
            double ms = (ros::Duration(current) - ros::Duration(start_time)).toSec() - s * 1000;
            p += "ELAPSED TIME: " + boost::lexical_cast<string>(m) + ":" + boost::lexical_cast<string>(s) + "." + 
                boost::lexical_cast<string>(ms) + "\n";
        }

        p += "ROUTE: ";

        for (int i = 0; i < route.size(); i++)
        {
            if (i == 0)
                p += route[i];
            else
                p += " - " + route[i];
        }

        p += "\nNEXT ROUTE: ";

        for (int i = 0; i < next_route.size(); i++)
        {
            if (i == 0)
                p += next_route[i];
            else
                p += " - " + next_route[i];
        }

        p += "\nSCORE: " + score;
        p += "\nPENALTIES: " + penalties;
        p += "\nSEMAPHORE: " + getSemaphoreName(last_semaphore) + "\n\n";

        cout << p;
    };
};

#endif