#include "ContactWaypointsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactWaypointsPlugin)

/////////////////////////////////////////////////
ContactWaypointsPlugin::ContactWaypointsPlugin() : SensorPlugin()
{
  // Publisher
  ros::NodeHandle n;
  this->pub = n.advertise<std_msgs::String>(WAYPOINTS_TOPIC, 1000);
}

/////////////////////////////////////////////////
ContactWaypointsPlugin::~ContactWaypointsPlugin()
{
}

/////////////////////////////////////////////////
void ContactWaypointsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactWaypointsPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactWaypointsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactWaypointsPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std_msgs::String msg;
    msg.data = buildMessage(WAYPOINT, contacts.contact(i).collision1(), contacts.contact(i).collision2());
    this->pub.publish(msg);
  }
}