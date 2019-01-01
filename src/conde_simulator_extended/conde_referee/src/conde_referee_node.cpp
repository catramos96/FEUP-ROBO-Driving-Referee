#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../resources/src/utils.h"
#include "../../resources/src/robot_resources.h"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

vector<Robot *> robots = vector<Robot *>();
SemaphoreState semaphoreState = UP;

vector<string> split(string s, string delimiter)
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  string token;
  vector<string> res;

  while ((pos_end = s.find(delimiter, pos_start)) != string::npos)
  {
    token = s.substr(pos_start, pos_end - pos_start);
    pos_start = pos_end + delim_len;
    res.push_back(token);
  }

  res.push_back(s.substr(pos_start));
  return res;
}

/*
// Message received format: robot_timeDone
void saveRobotTimeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  string message = msg->data.c_str();
  vector<string> results = split(message, "_");

  // Save robot name and time done
  Robot robot = {results[0], atof(results[1].c_str())};
  robots.push_back(robot);
}

// To be used when the race finishes
void printTimesCallback()
{
  int size = robots.size();
  for (int i = 0; i < size; i++)
  {
    std::cout << "Robot: " << robots[i].name << " - Time Done: " << robots[i].time;
    std::cout << "\n";
  }
}
*/

void GeneralizedCallback(const std_msgs::String::ConstPtr &msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  cout << msg->data.c_str() << endl;
}

void SemaphoreCallback(const std_msgs::String::ConstPtr &msg)
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

  cout << m << " " << semaphoreState << "\n";
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

    //check if waypoint will be added to route
    bool add_w = false;

    if (r->route.size() == 0)
      add_w = true;
    else if (r->getLastWaypoint() != waypoint)
      add_w = true;

    if (add_w)
    {
      //started race?
      if (waypoint == START_WAYPOINT && r->start_time == -1)
      {
        r->start_race();
        cout << "START" << endl;
      }

      r->addWaypoint(waypoint);

      //print robot info
      r->print();
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
  ros::NodeHandle n5;

  //ros::Subscriber sub1 = n1.subscribe(ROBOT_TIME_TOPIC, 1, saveRobotTimeCallback);
  //ros::Subscriber sub2 = n2.subscribe(BOUNDARIES_TOPIC, 1, GeneralizedCallback); //Change
  ros::Subscriber sub3 = n3.subscribe(WAYPOINTS_TOPIC, 1, WaypointsCallback);
  //ros::Subscriber sub4 = n4.subscribe(SEMAPHORE_TOPIC, 1, GeneralizedCallback); //Change
  //ros::Subscriber sub5 = n5.subscribe(SEMAPHORE_STATE_TOPIC, 1, SemaphoreCallback);

  //ROS_INFO("Listening on /conde_referee_robot_time");

  ros::spin();

  return 0;
}