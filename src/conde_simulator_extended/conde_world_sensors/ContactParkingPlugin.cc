#include "ContactParkingPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactParkingPlugin)

/////////////////////////////////////////////////
ContactParkingPlugin::ContactParkingPlugin() : SensorPlugin()
{
  // Publisher
  ros::NodeHandle n;
  this->pub = n.advertise<std_msgs::String>(BOUNDARIES_TOPIC, 1000);
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

    std_msgs::String str;
    str.data = "Boundaries: " + contacts.contact(i).collision1() + " " + contacts.contact(i).collision2();
    this->pub.publish(str);

    std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      /*std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n" << "OLIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII";*/
    }
  }
}