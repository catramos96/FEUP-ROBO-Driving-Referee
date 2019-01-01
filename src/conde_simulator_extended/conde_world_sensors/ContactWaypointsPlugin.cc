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
    gzerr << "ContactWaypointsPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactWaypointsPlugin::OnUpdate, this));

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
                                                      boost::bind(&ContactWaypointsPlugin::SemaphoreStateCallback, this, _1),
                                                      ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&ContactWaypointsPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ContactWaypointsPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std_msgs::String str;
    str.data = "Waypoints: " + contacts.contact(i).collision1() + " " + contacts.contact(i).collision2() + "\n";
    this->pub.publish(str);

    /*
    TODO: Create messages with id/name of the objects that collided and the time.
    */
  }
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg String that represents the semaphore's state
void ContactWaypointsPlugin::SemaphoreStateCallback(const std_msgs::String::ConstPtr &_msg)
{
  semaphore_state = _msg->data.c_str();
  ROS_INFO("Semaphore State: [%s]", _msg->data.c_str());
}

/// \brief ROS helper function that processes messages
void ContactWaypointsPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}