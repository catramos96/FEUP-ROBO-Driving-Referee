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
using namespace boost;

class RobotCollision
{
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
    double last_penalty_time;
    double parking_time;
    vector<int> route;
    vector<int> next_route;                       //route to be performed
    vector<RobotCollision> inside_collisions;     // info on wheter the robot is inside the track
    vector<RobotCollision> boundaries_collisions; // info on wheter the robot is touching the boundaries
    vector<RobotCollision> outside_collisions;    // info on wheter the robot is outside the track
    bool had_boundary_collision;
    double driving_score;
    double parking_score;
    int driving_penalties;
    int parking_penalties;
    bool is_inside_parking;
    bool parking_started;
    Sensor last_penalty;
    RaceState race_state;

  public:
    Robot(string name)
    {
        driving_score = 0;
        parking_score = 0;
        driving_penalties = 0;
        parking_penalties = 0;
        setName(name);
        next_route.push_back(START_WAYPOINT);

        // TO DELETE , IT'S FOR TEST
        /*setRaceState(BAY_PARKING);
        parking_started = false;
        route.push_back(6);
        next_route.push_back(9);*/

        parking_started = false;
        setBoundaryCollision(false);
        initializeCollisionVectors();
    };
    void setName(string new_name) { name = new_name; };
    void startRace()
    {
        start_time = ros::Time::now().toSec();
        race_state = ONGOING;
    };
    void endDrivingChallenge() { end_time = ros::Time::now().toSec(); };
    void addDrivingPenalty(int new_penalty)
    {
        driving_penalties += new_penalty;
        last_penalty_time = ros::Time::now().toSec();
    };
    void addParkingPenalty(int new_penalty)
    {
        parking_penalties += new_penalty;
        last_penalty_time = ros::Time::now().toSec();
    };
    void setBoundaryCollision(bool state) { had_boundary_collision = state; };
    void setInsideParking(bool state) { is_inside_parking = state; };
    void startParking()
    {
        parking_started = true;
        parking_time = ros::Time::now().toSec();
    };
    void setRaceState(RaceState state) { race_state = state; };
    void setLastPenalty(Sensor sensor) { last_penalty = sensor; };
    void setParkingScore(double score) { parking_score = score; }
    RaceState getRaceState() { return race_state; };
    Sensor getLastPenalty() { return last_penalty; };
    double getLastPenaltyTime() { return last_penalty_time; };
    double getParkingTime() { return parking_time; };
    bool getHadBoundaryCollision() { return had_boundary_collision; };
    bool isParking() { return parking_started; };
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
    };
    void calculateDrivingScore()
    { // Driving with signs
        int half_laps = LAPS * 2;
        driving_score = half_laps * DRIVING_SCORE_REFERENCE + (half_laps * TIME_REFERENCE - end_time) - driving_penalties;
    };
    void calculateParkingScore()
    { // Parallel or Bay parking without obstacles
        parking_score = PARKING_SCORE_REFERENCE - parking_penalties;
    };
    void consumeRouteWaypoint(int waypoint, SemaphoreState state)
    {
        if (race_state == DISQUALIFIED || race_state == FINISHED)
            return;

        // Keeps refreshing flag in case robot had left the parking
        // No more waypoints, referee is just waiting for the robot to park
        if (isParkingWaypoint(waypoint) && isParking())
        {
            setInsideParking(true);

            double current = ros::Time::now().toSec();

            // Robot has 5 seconds to finish the parking
            if (current - parking_time > 5)
            {
                calculateParkingScore();
                setRaceState(FINISHED);
                print();
            }
            return;
        }

        int last_waypoint = getLastWaypoint();

        // No need to double check unless is a parking waypoint - Several messages are received during the time the robot passes through the sensor
        if (waypoint == last_waypoint)
            return;

        int next_waypoint = START_WAYPOINT;

        if (next_route.size() != 0)
            next_waypoint = next_route[0];
        else
        {
            next_route.clear();
            next_route.push_back(START_WAYPOINT);
            next_waypoint = START_WAYPOINT;
        }

        //correct path
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
                    // Check time of last semaphore penalty
                    if (last_penalty == SEMAPHORE)
                    {
                        double current = ros::Time::now().toSec();

                        // After 2 seconds it's okay to penalty again
                        if (current - last_penalty_time > 2)
                        {
                            addDrivingPenalty(60);
                            last_penalty = SEMAPHORE;
                        }
                    }
                    else
                    {
                        addDrivingPenalty(60);
                        last_penalty = SEMAPHORE;
                    }
                }

                if (state == PARK)
                {
                    endDrivingChallenge();
                    calculateDrivingScore();
                    setRaceState(PARALLEL_PARKING);
                }

                next_route = getNextRoute(state, goesToRight(last_waypoint));
            }

            //add waypoint to route
            route.push_back(next_waypoint);

            //remove waypoint from next route if not removed
            //when route was updated (near semaphore)
            if (next_route[0] == next_waypoint)
                next_route.erase(next_route.begin());

            if (isParkingWaypoint(waypoint) && !isParking())
            {
                setInsideParking(true);
                startParking();
                cout << "Starting parking..." << endl;
            }

            //finish route
            if (hasFinishRace(route))
            {
                // Bay parking challenge starts after driving challenge is done
                if (race_state != PARALLEL_PARKING)
                {
                    endDrivingChallenge();
                    calculateDrivingScore();
                    setRaceState(BAY_PARKING);
                    next_route.push_back(9);
                }
            }

            print();
        }
        else if (last_waypoint == START_WAYPOINT) // Wrong direction after semaphore
        {
            // Check time of last semaphore penalty
            if (last_penalty == SEMAPHORE)
            {
                double current = ros::Time::now().toSec();

                // After 2 seconds it's okay to penalty again
                if (current - last_penalty_time > 2)
                {
                    addDrivingPenalty(25);
                    last_penalty = SEMAPHORE;
                }
            }
            else
            {
                addDrivingPenalty(25);
                last_penalty = SEMAPHORE;
            }
        }
    };
    void initializeCollisionVectors()
    {
        RobotCollision right_wheel = RobotCollision("right_wheel", false);
        RobotCollision left_wheel = RobotCollision("left_wheel", false);
        RobotCollision front_wheel = RobotCollision("chassis", false);

        inside_collisions.push_back(right_wheel);
        inside_collisions.push_back(left_wheel);
        inside_collisions.push_back(front_wheel);

        outside_collisions.push_back(right_wheel);
        outside_collisions.push_back(left_wheel);
        outside_collisions.push_back(front_wheel);

        boundaries_collisions.push_back(right_wheel);
        boundaries_collisions.push_back(left_wheel);
        boundaries_collisions.push_back(front_wheel);
    };
    bool setCollisionStateBySensor(string component, Sensor sensor, bool state)
    {
        switch (sensor)
        {
        case TRACK_OUTSIDE:

            for (int i = 0; i < outside_collisions.size(); i++)
            {
                if (component.compare(outside_collisions[i].component) == 0)
                {
                    outside_collisions[i].setCollisionState(state);
                    return true;
                }
            }
            break;
        case TRACK_INSIDE:
            for (int i = 0; i < inside_collisions.size(); i++)
            {
                if (component.compare(inside_collisions[i].component) == 0)
                {
                    inside_collisions[i].setCollisionState(state);
                    return true;
                }
            }
            break;
        case TRACK_BOUNDS:
            for (int i = 0; i < boundaries_collisions.size(); i++)
            {
                if (component.compare(boundaries_collisions[i].component) == 0)
                {
                    boundaries_collisions[i].setCollisionState(state);
                    return true;
                }
            }
            break;
        }

        return false;
    };
    bool isInsideTrack()
    {
        for (int i = 0; i < inside_collisions.size(); i++)
        {
            if (inside_collisions[i].isColliding == false)
            {
                return false;
            }
        }

        return true;
    };
    bool isOutsideTrack()
    {
        for (int i = 0; i < outside_collisions.size(); i++)
        {
            if (outside_collisions[i].isColliding == false)
            {
                return false;
            }
        }

        return true;
    };
    void print(void)
    {
        string p = "NAME: " + name + "\n";
        p += "LAPS: " + lexical_cast<string>(getCurrentLap(route)) + lexical_cast<string>("/") + lexical_cast<string>(LAPS) + "\n";
        if (race_state == FINISHED)
        {
            p += "STATUS: " + getRaceStateName(race_state) + "\n";
            double secs = (ros::Duration(end_time) - ros::Duration(start_time)).toSec();
            double mils = (int)((secs - (int)secs) * 1000);
            secs = secs - mils / 1000;
            double mins = (int)(secs / 60);
            secs = (int)(secs - mins * 60);

            p += "ELAPSED TIME: " + lexical_cast<string>(mins) + ":" + lexical_cast<string>(secs) + "." +
                 lexical_cast<string>(mils) + "\n";
        }
        else
        {
            p += "STATUS: " + getRaceStateName(race_state) + "\n";

            double current = ros::Time::now().toSec();
            double secs = (ros::Duration(current) - ros::Duration(start_time)).toSec();
            double mils = (int)((secs - (int)secs) * 1000);
            secs = secs - mils / 1000;
            double mins = (int)(secs / 60);
            secs = (int)(secs - mins * 60);

            p += "ELAPSED TIME: " + lexical_cast<string>(mins) + ":" + lexical_cast<string>(secs) + "." +
                 lexical_cast<string>(mils) + "\n";
        }

        p += "ROUTE: ";

        for (int i = 0; i < route.size(); i++)
        {
            if (i == 0)
                p += lexical_cast<string>(route[i]);
            else
                p += " - " + lexical_cast<string>(route[i]);
        }

        p += "\nNEXT ROUTE: ";

        for (int i = 0; i < next_route.size(); i++)
        {
            if (i == 0)
                p += lexical_cast<string>(next_route[i]);
            else
                p += " - " + lexical_cast<string>(next_route[i]);
        }

        p += "\nDRIVING PENALTIES: " + lexical_cast<string>(driving_penalties);

        if (race_state == FINISHED)
        {
            p += "\nPARKING PENALTIES: " + lexical_cast<string>(parking_penalties);
            p += "\n\nDRIVING SCORE: " + lexical_cast<string>(driving_score);
            p += "\n\nPARKING SCORE: " + lexical_cast<string>(parking_score);
        }

        cout << p;
    };
};

#endif