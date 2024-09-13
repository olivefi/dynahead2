#include "dynahead2_control/Dynahead2Control.hpp"

namespace dynahead2_control {

Dynahead2Control::Dynahead2Control(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool Dynahead2Control::init() {
  updateRate_ = param<double>("update_rate", 100);
  deviceName_ = param<std::string>("device_name", "/dev/ttyUSB0");
  encoderTicks_ = param<uint>("encoder_ticks", 4096);
  
  dynamixels_["pitch"] = param<int>("pitch/id", PITCH_DXL_ID);
  dynamixels_["yaw"] = param<int>("yaw/id", YAW_DXL_ID);
  jointOffsets_["pitch"] = param<double>("pitch/offset", 1.57);
  jointOffsets_["yaw"] = param<double>("yaw/offset", 1.57);
  jointDirections_["pitch"] = param<int>("pitch/direction", 1);
  jointDirections_["yaw"] = param<int>("yaw/direction", -1);
  jointLimits_["pitch"] = param<std::vector<double>>("pitch/limits", {-M_PI/4., M_PI/4.});
  jointLimits_["yaw"] = param<std::vector<double>>("yaw/limits", {-M_PI/4., M_PI/4.});

  jointStateCmd_ = sensor_msgs::JointState();
  jointStateCmd_.name = {"pitch", "yaw"};
  jointStateCmd_.position = {0, 0};

  if (!initDynamixels(dynamixels_)) {
    ROS_ERROR("Failed to initialize Dynamixels");
    return false;
  }

  jointStatePub_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 1);
  jointCmdSub_ = nh_->subscribe("joint_commands", 1, &Dynahead2Control::jointCmdCallback, this);

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&Dynahead2Control::update, this, _1);
  workerOptions.timeStep_ = 1.0 / updateRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void Dynahead2Control::cleanup() {
  portHandler_->closePort();
  jointStatePub_.shutdown();
}

bool Dynahead2Control::update(const any_worker::WorkerEvent &event) {
  jointState_ = sensor_msgs::JointState();
  if (!getPositions(jointState_)) {
    return false;
  }
  jointState_.header.stamp = ros::Time::now();
  jointStatePub_.publish(jointState_);
  if (!setPositions(jointStateCmd_)) {
    return false;
  }
  return true;
}

bool Dynahead2Control::initDynamixels(const std::map<std::string, int> &dynamixels) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler_ = dynamixel::PortHandler::getPortHandler(deviceName_.c_str());
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler_->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return false;
  }

  if (!portHandler_->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return false;
  }

  for (const auto& [dynamixel_name, dynamixel_id] : dynamixels) {
    dxl_comm_result = packetHandler_->write1ByteTxRx(
      portHandler_, dynamixel_id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR_STREAM("Failed to disable torque for Dynamixel ID " << dynamixel_id);
      return false;
    }
  }
  return true;
}

bool Dynahead2Control::getPositions(sensor_msgs::JointState &jointState) {
  for (const auto& [dynamixel_name, dynamixel_id] : dynamixels_) {
    uint32_t position;
    if (!readPosition(dynamixel_id, position)) {
      return false;
    }
    jointState.name.push_back(dynamixel_name);
    jointState.position.push_back(toPlusMinusPi(jointDirections_[dynamixel_name] * (position/(encoderTicks_/(2*M_PI))) - jointOffsets_[dynamixel_name]));
  }
  return true;
}

bool Dynahead2Control::setPositions(const sensor_msgs::JointState &jointStateCmd) {
  for (int i = 0; i < jointStateCmd.name.size(); i++) {
    const auto dynamixel_name = jointStateCmd.name[i];
    const auto dynamixel_id = dynamixels_[dynamixel_name];
    double target = jointStateCmd.position[i];
    if (target < jointLimits_[dynamixel_name][0] || target > jointLimits_[dynamixel_name][1]) {
      ROS_WARN_STREAM_THROTTLE(0.5, "Goal position for Dynamixel ID " << dynamixel_id << " is out of bounds. Value: " << target
        << ". Clamping to [" << jointLimits_[dynamixel_name][0] << ", " << jointLimits_[dynamixel_name][1] << "]");
      target = std::clamp(target, jointLimits_[dynamixel_name][0], jointLimits_[dynamixel_name][1]);
    }
    const int goal_position = (jointDirections_[dynamixel_name] * target + jointOffsets_[dynamixel_name]) * (encoderTicks_/(2*M_PI));
    if (!setPosition(dynamixel_id, goal_position)) {
      return false;
    }
  }
  return true;
}

bool Dynahead2Control::readPosition(const int &dynamixel_id, uint32_t &position){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->read4ByteTxRx(
    portHandler_, dynamixel_id, ADDR_PRESENT_POSITION, &position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM_THROTTLE(1, "Failed to read present position for Dynamixel ID "<< dynamixel_id);
    return false;
  }
  return true;
}

bool Dynahead2Control::setPosition(const int &dynamixel_id, const uint32_t &goal_position) {
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write4ByteTxRx(
    portHandler_, dynamixel_id, ADDR_GOAL_POSITION, goal_position, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM_THROTTLE(1, "Failed to write goal position for Dynamixel ID "<< dynamixel_id);
    return false;
  }
  return true;
}

void Dynahead2Control::jointCmdCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  jointStateCmd_ = *msg;
}

} /* namespace dynahead2_control */
