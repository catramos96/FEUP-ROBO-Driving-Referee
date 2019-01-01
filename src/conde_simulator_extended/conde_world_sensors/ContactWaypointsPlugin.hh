#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "../utils.hh"

namespace gazebo
{
/// \brief An example plugin for a contact sensor.
class ContactWaypointsPlugin : public SensorPlugin
{
  /// \brief Constructor.
public:
  ContactWaypointsPlugin();

  /// \brief Destructor.
public:
  virtual ~ContactWaypointsPlugin();

  /// \bried Semaphore State Variable
private:
  std::string semaphore_state;

  /// \brief Load the sensor plugin.
  /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
public:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  /// \brief Callback that receives the contact sensor's update signal.
private:
  virtual void OnUpdate();

  /// \brief Pointer to the contact sensor
private:
  sensors::ContactSensorPtr parentSensor;

  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnUpdate callback.
private:
  event::ConnectionPtr updateConnection;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

public:
  void SemaphoreStateCallback(const std_msgs::String::ConstPtr &msg);

private:
  void QueueThread();

public:
  ros::Publisher pub;
};
} // namespace gazebo
#endif