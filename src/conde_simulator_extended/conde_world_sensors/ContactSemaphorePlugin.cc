#include "ContactSemaphorePlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactSemaphorePlugin)

/////////////////////////////////////////////////
ContactSemaphorePlugin::ContactSemaphorePlugin() : SensorPlugin()
{
  // Publisher
  ros::NodeHandle n;
  this->pub = n.advertise<std_msgs::String>(SEMAPHORE_TOPIC, 1000);
}

/////////////////////////////////////////////////
ContactSemaphorePlugin::~ContactSemaphorePlugin()
{
}

/////////////////////////////////////////////////
void ContactSemaphorePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactSemaphorePlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactSemaphorePlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactSemaphorePlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std_msgs::String msg;
    msg.data = (SEMAPHORE, contacts.contact(i).collision1(), contacts.contact(i).collision2());
    this->pub.publish(msg);
  }
}