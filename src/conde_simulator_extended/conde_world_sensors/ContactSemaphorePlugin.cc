#include "ContactSemaphorePlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactSemaphorePlugin)

/////////////////////////////////////////////////
ContactSemaphorePlugin::ContactSemaphorePlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactSemaphorePlugin::~ContactSemaphorePlugin()
{
}

/////////////////////////////////////////////////
void ContactSemaphorePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

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

    // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::String>("/conde_signalling_panel_state", 1,
        boost::bind(&ContactSemaphorePlugin::SemaphoreStateCallback, this, _1),
        ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&ContactSemaphorePlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ContactSemaphorePlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    /*std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";*/

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
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n" << "OLEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE";*/
    }
  }
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg String that represents the semaphore's state
void ContactSemaphorePlugin::SemaphoreStateCallback(const std_msgs::String::ConstPtr &_msg)
{
  semaphore_state = _msg->data.c_str();
  ROS_INFO("Semaphore State: [%s]", _msg->data.c_str());
}

/// \brief ROS helper function that processes messages
void ContactSemaphorePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}