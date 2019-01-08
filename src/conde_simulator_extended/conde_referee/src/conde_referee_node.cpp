#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../resources/src/utils.h"
#include "../../resources/src/robot_resources.h"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

vector<Robot *> robots;
SemaphoreState semaphoreState = UP;

Robot *getRobotByName(string name)
{
  for (int i = 0; i < robots.size(); i++)
  {
    if (name.compare(robots[i]->name) == 0)
    {
      return robots[i];
    }
  }

  return NULL;
}

void BayParkingBoundsCallback(const std_msgs::String::ConstPtr &msg)
{
  string m = msg->data.c_str();

  vector<string> parts = vector<string>();
  split_regex(parts, m, regex(SEPARATOR));

  // check if image is of correct type
  if (parts[0].compare(getSensorName(PARKING)))
    return;
  else
  {
    // get message information
    string robotName = parts[1];

    // check if robot exists
    Robot *r = getRobotByName(robotName);

    // new robot
    if (r == NULL)
    {
      r = new Robot(robotName);
      robots.push_back(r);
    }

    // If the robot has been disqualified there is no need to check collisions
    RaceState race_state = r->getRaceState();
    if (race_state == DISQUALIFIED || race_state == PARALLEL_PARKING)
      return;

    if (r->isParking())
    {
      if (r->getLastPenalty() == PARKING)
      {
        double current = ros::Time::now().toSec();

        // After 1 second it's okay to penalty again
        // To handle a lot of messages sent by the sensor
        if (current - r->getLastPenaltyTime() >= PARKING_PENALTY_TIME_INTERVAL)
        {
          r->addParkingPenalty(10);
          r->setLastPenalty(PARKING);
          cout << "\n\nParking Penalty for robot: " << r->name << endl;
        }
      }
      else
      {
        r->addParkingPenalty(10);
        r->setLastPenalty(PARKING);
        cout << "\n\nParking Penalty for robot: " << r->name << endl;
      }
    }
  }
}

void SemaphoreStateCallback(const std_msgs::String::ConstPtr &msg)
{
  string m = msg->data.c_str();

  if (m.compare("LEFT") == 0)
  {
    semaphoreState = LEFT;
  }
  else if (m.compare("RIGHT") == 0)
  {
    semaphoreState = RIGHT;
  }
  else if (m.compare("UP") == 0)
  {
    semaphoreState = UP;
  }
  else if (m.compare("STOP") == 0)
  {
    semaphoreState = STOP;
  }
  else if (m.compare("PARK") == 0)
  {
    semaphoreState = PARK;
  }

  cout << "SEMAPHORE: " << semaphoreState << endl;
}

void WaypointsCallback(const std_msgs::String::ConstPtr &msg)
{
  string m = msg->data.c_str();

  vector<string> parts = vector<string>();
  split_regex(parts, m, regex(SEPARATOR));

  // check if image is of correct type
  if (parts[0].compare(getSensorName(WAYPOINT)) != 0)
    return;
  else
  {
    // get message information
    string name = parts[1];
    int waypoint = atoi(parts[2].c_str());

    // check if robot exists
    Robot *r = getRobotByName(name);

    // new robot
    if (r == NULL)
    {
      r = new Robot(name);
      robots.push_back(r);
    }

    // consume waypoint if relevant
    r->consumeRouteWaypoint(waypoint, semaphoreState);
  }
}

void BoundariesCallback(const std_msgs::String::ConstPtr &msg)
{
  string m = msg->data.c_str();

  vector<string> parts = vector<string>();
  split_regex(parts, m, regex(SEPARATOR));

  // check if image is of correct type
  if (parts[0].compare(getSensorName(TRACK)))
    return;
  else
  {
    // get message information
    Sensor sensorType = getSensor(parts[1]);
    string robotName = parts[2];
    string robotComponent = parts[3];

    // check if robot exists
    Robot *r = getRobotByName(robotName);

    // new robot
    if (r == NULL)
    {
      r = new Robot(robotName);
      robots.push_back(r);
    }

    RaceState race_state = r->getRaceState();

    // If the robot has been disqualified there is no need to check collisions
    if (race_state == DISQUALIFIED)
      return;

    switch (sensorType)
    {
    case TRACK_OUTSIDE:
      r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, true);
      r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, false);
      r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, false);

      // Flag to know if robot is inside the parking space
      if (r->isParking() && r->isOutsideTrack())
      {
        r->setInsideParking(false);
        double current = ros::Time::now().toSec();

        // Robot has 5 seconds to finish the parking
        if (current - r->getParkingTime() >= PARKING_TIME)
        {
          cout << "\n\n"
               << r->name << " failed the parking challenge!" << endl;
          r->setParkingScore(0);
          r->endParking();
          r->setRaceState(FINISHED);
          r->print();
        }
        return;
      }

      // Outside of the track penalty
      if (race_state == ONGOING && r->isOutsideTrack())
      {
        r->addDrivingPenalty(20);
        r->setRaceState(DISQUALIFIED);
        r->setLastPenalty(TRACK_OUTSIDE);
        cout << "\n\n"
             << r->name << " DISQUALIFIED!" << endl;
      }
      break;
    case TRACK_INSIDE:
      r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, true);
      r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, false);
      r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, false);

      // Flag to know if robot is inside the parking space
      /*if (r->isParking() && r->isInsideTrack())
      {
        r->setInsideParking(false);
        double current = ros::Time::now().toSec();

        // Robot has 5 seconds to finish the parking
        if (current - r->getParkingTime() >= PARKING_TIME)
        {
          r->setParkingScore(0); // Failed to parking
          r->endParking();
          r->setRaceState(FINISHED);
          r->print();
        }
        return;
      }*/

      // Penalty added when robot is back to the track
      if (race_state == ONGOING && r->isInsideTrack() && r->getHadBoundaryCollision())
      {
        if (r->getLastPenalty() == TRACK_BOUNDS)
        {
          double current = ros::Time::now().toSec();

          // After 2 seconds it's okay to penalty again
          // To handle a lot of messages sent by the sensor
          if (current - r->getLastPenaltyTime() >= BOUNDS_PENALTY_TIME_INTERVAL)
          {
            r->addDrivingPenalty(10);
            r->setBoundaryCollision(false);
            r->setLastPenalty(TRACK_BOUNDS);
            cout << "\n\nBoundaries Penalty for robot: " << r->name << endl;
          }
        }
        else
        {
          r->addDrivingPenalty(10);
          r->setBoundaryCollision(false);
          r->setLastPenalty(TRACK_BOUNDS);
          cout << "\n\nBoundaries Penalty for robot: " << r->name << endl;
        }
      }

      break;
    case TRACK_BOUNDS:
      r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, true);
      r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, false);
      r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, false);
      r->setBoundaryCollision(true);

      // For parallel parking - Parking limit is the track bounds
      if (r->isParking())
      {
        if (r->getLastPenalty() == TRACK_BOUNDS)
        {
          double current = ros::Time::now().toSec();

          // After 1 second it's okay to penalty again
          // To handle a lot of messages sent by the sensor
          if (current - r->getLastPenaltyTime() >= PARKING_PENALTY_TIME_INTERVAL)
          {
            r->addParkingPenalty(10);
            r->setLastPenalty(TRACK_BOUNDS);
            cout << "\n\nParking Penalty for robot: " << r->name << endl;
          }
        }
        else
        {
          r->addParkingPenalty(10);
          r->setLastPenalty(TRACK_BOUNDS);
          cout << "\n\nParking Penalty for robot: " << r->name << endl;
        }
      }
      
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conde_referee");

  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::NodeHandle n4;

  ros::Subscriber sub1 = n1.subscribe(BOUNDARIES_TOPIC, 1, BoundariesCallback);
  ros::Subscriber sub2 = n2.subscribe(WAYPOINTS_TOPIC, 1, WaypointsCallback);
  ros::Subscriber sub3 = n3.subscribe(SEMAPHORE_STATE_TOPIC, 1, SemaphoreStateCallback);
  ros::Subscriber sub4 = n4.subscribe(PARK_TOPIC, 1, BayParkingBoundsCallback);

  //ROS_INFO("Listening on /conde_referee_robot_time");

  ros::spin();

  return 0;
}