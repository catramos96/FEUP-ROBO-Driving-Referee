#ifndef _ROBOT_H_
#define _ROBOT_H_

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "boost/lexical_cast.hpp"

#include "utils.h"
#include "logic.h"
#include "robot_collision.h"

using namespace std;
using namespace boost;

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
    Robot(string name);
    void initializeCollisionVectors();
    void setName(string new_name);

    void startRace();
    void endDrivingChallenge();

    void addDrivingPenalty(int new_penalty);
    void addParkingPenalty(int new_penalty);

    void setBoundaryCollision(bool state);
    void setInsideParking(bool state);

    void startParking();
    void endParking();

    void setRaceState(RaceState state);
    void setLastPenalty(Sensor sensor);
    void setParkingScore(double score);

    RaceState getRaceState();
    Sensor getLastPenalty();
    double getLastPenaltyTime();
    double getParkingTime();
    bool getHadBoundaryCollision();
    bool isParking();
    int getLastWaypoint();
    double getRaceTime();

    void calculateDrivingScore();
    void calculateParkingScore();

    void consumeRouteWaypoint(int waypoint, SemaphoreState state);
    
    bool setCollisionStateBySensor(string component, Sensor sensor, bool state);
    bool isInsideTrack();
    bool isOutsideTrack();
    void print(void);
};

#endif