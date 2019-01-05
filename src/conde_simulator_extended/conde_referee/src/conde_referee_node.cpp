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

Robot* getRobotByName(string name) {
  for (int i = 0; i < robots.size(); i++)
  {
    if (name.compare(robots[i]->name) == 0)
    {
      return robots[i];
    }
  }

  return NULL;
}

void GeneralizedCallback(const std_msgs::String::ConstPtr &msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  cout << msg->data.c_str() << endl;
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

    Robot *r = new Robot(name);
    bool found = false;

    // check if robot exists
    for (int i = 0; i < robots.size(); i++)
    {
      if (name.compare(robots[i]->name) == 0)
      {
        r = robots[i];
        found = true;
        break;
      }
    }

    // new robot
    if (found == false)
    {
      r->setName(name);
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
    if(r == NULL) {
      r = new Robot(robotName);
      robots.push_back(r);
    }

    // If the robot has been disqualified there is no need to check collisions
    if(r->getRaceState() == DISQUALIFIED)
      return;

    switch (sensorType)
    {
      case TRACK_OUTSIDE:
          r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, true);
          r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, false);
          r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, false);

          if(r->isOutsideTrack()) {
            r->addDrivingPenalty(20);
            r->setRaceState(DISQUALIFIED);
            r->setLastPenalty(TRACK_OUTSIDE);
            cout << r->name << " DISQUALIFIED!" << endl;
          }
          break;
      case TRACK_INSIDE:
          r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, true);
          r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, false);
          r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, false);

          // Penalty added when robot is back to the track
          if(r->isInsideTrack() && r->getHadBoundaryCollision()) {
            if(r->getLastPenalty() == TRACK_BOUNDS) {
              double current = ros::Time::now().toSec();

              // After 2 seconds it's okay to penalty again
              // To handle a lot of messages sent by the sensor
              if(current - r->getLastPenaltyTime() > 2) {
                r->addDrivingPenalty(10);
                r->setBoundaryCollision(false);
                r->setLastPenalty(TRACK_BOUNDS);
                cout << "PENALTY for robot: " << r->name << endl;
              }
            } else {
              r->addDrivingPenalty(10);
              r->setBoundaryCollision(false);
              r->setLastPenalty(TRACK_BOUNDS);
              cout << "PENALTY for robot: " << r->name << endl;
            }
          }

          break;
      case TRACK_BOUNDS:
          r->setCollisionStateBySensor(robotComponent, TRACK_BOUNDS, true);
          r->setCollisionStateBySensor(robotComponent, TRACK_OUTSIDE, false);
          r->setCollisionStateBySensor(robotComponent, TRACK_INSIDE, false);
          r->setBoundaryCollision(true);

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
  //ros::Subscriber sub4 = n4.subscribe(SEMAPHORE_TOPIC, 1, SemaphoreCallback);
  ros::Subscriber sub3 = n3.subscribe(SEMAPHORE_STATE_TOPIC, 1, SemaphoreStateCallback);
  ros::Subscriber sub4 = n4.subscribe(PARK_TOPIC, 1, GeneralizedCallback);

  //ROS_INFO("Listening on /conde_referee_robot_time");

  ros::spin();

  return 0;
}