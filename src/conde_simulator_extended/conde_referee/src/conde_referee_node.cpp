#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../../utils.hh"

#include <string>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

struct RobotTime
{
  string name;
  double time;
};

vector<RobotTime> robotsTimeArray;
SemaphoreState semaphoreState;

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

// Message received format: robot_timeDone
void saveRobotTimeCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  string message = msg->data.c_str();
  vector<string> results = split(message, "_");

  // Save robot name and time done
  RobotTime robot = {results[0], atof(results[1].c_str())};
  robotsTimeArray.push_back(robot);
}

// To be used when the race finishes
void printTimesCallback()
{
  int size = robotsTimeArray.size();
  for (int i = 0; i < size; i++)
  {
    std::cout << "Robot: " << robotsTimeArray[i].name << " - Time Done: " << robotsTimeArray[i].time;
    std::cout << "\n";
  }
}

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conde_referee");

  ros::NodeHandle n1;
  ros::NodeHandle n2;
  ros::NodeHandle n3;
  ros::NodeHandle n4;
  ros::NodeHandle n5;

  ros::Subscriber sub1 = n1.subscribe(ROBOT_TIME_TOPIC, 1, saveRobotTimeCallback);
  ros::Subscriber sub2 = n2.subscribe(BOUNDARIES_TOPIC, 1, GeneralizedCallback); //Change
  ros::Subscriber sub3 = n3.subscribe(WAYPOINTS_TOPIC, 1, GeneralizedCallback);  //Change
  ros::Subscriber sub4 = n4.subscribe(SEMAPHORE_TOPIC, 1, GeneralizedCallback);  //Change
  ros::Subscriber sub5 = n5.subscribe(SEMAPHORE_STATE_TOPIC, 1, SemaphoreCallback);

  ROS_INFO("Listening on /conde_referee_robot_time");

  ros::spin();

  return 0;
}