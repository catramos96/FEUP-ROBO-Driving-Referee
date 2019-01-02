#include "ContactParkingPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactParkingPlugin)

/////////////////////////////////////////////////
ContactParkingPlugin::ContactParkingPlugin() : SensorPlugin()
{
  // Publisher
  ros::NodeHandle n;
  this->pub = n.advertise<std_msgs::String>(PARK_TOPIC, 1000);
}

/////////////////////////////////////////////////
ContactParkingPlugin::~ContactParkingPlugin()
{
}

/////////////////////////////////////////////////
void ContactParkingPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactParkingPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactParkingPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactParkingPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << contacts.contact(i).collision1() + " " + contacts.contact(i).collision2();
    //std_msgs::String msg;
    //msg.data = contacts.contact(i).collision1() + " " contacts.contact(i).collision2();
    //buildMessage(PARKING, contacts.contact(i).collision1(), contacts.contact(i).collision2());
    //this->pub.publish(msg);
  }
}