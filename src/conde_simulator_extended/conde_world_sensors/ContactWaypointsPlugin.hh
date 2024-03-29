#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include "utils.h"

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

public:
  ros::Publisher pub;
};
} // namespace gazebo
#endif