#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <sensor_msgs/JointState.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace dynahead2_control {

// Control table address
const int ADDR_TORQUE_ENABLE = 64;
const int ADDR_GOAL_POSITION = 116;
const int ADDR_PRESENT_POSITION = 132;

// Protocol version
const float PROTOCOL_VERSION = 2.0;

// Default setting
const int PITCH_DXL_ID = 1;
const int YAW_DXL_ID = 2;
const int BAUDRATE =  57600;

class Dynahead2Control : public any_node::Node {
public:
  Dynahead2Control(any_node::Node::NodeHandlePtr nh);
  ~Dynahead2Control() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent &event);

protected:
  // Methods
  bool initDynamixels(const std::map<std::string, int> &dynamixels);
  bool getPositions(sensor_msgs::JointState &jointState);
  bool setPositions(const sensor_msgs::JointState &jointStateCmd);

  bool readPosition(const int &dynamixel_id, uint32_t &position);
  bool setPosition(const int &dynamixel_id, const uint32_t &goal_position);

  // Externally settable variables
  double updateRate_;
  std::string deviceName_;
  uint encoderTicks_;
  std::map<std::string, double> jointOffsets_;
  std::map<std::string, int> jointDirections_;
  std::map<std::string, std::vector<double>> jointLimits_;

  // Internal state variables
  sensor_msgs::JointState jointState_;
  sensor_msgs::JointState jointStateCmd_;

  // Dynamixel Objects
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  std::map<std::string, int> dynamixels_;

  // Callbacks
  void jointCmdCallback(const sensor_msgs::JointState::ConstPtr &msg);

  // Publishers + Subscribers
  ros::Publisher jointStatePub_;
  ros::Subscriber jointCmdSub_;

  // Helper methods
  double toPlusMinusPi(double angle) {
    if (angle > M_PI) {
      while (angle > M_PI) {
        angle -= 2*M_PI;
      }
    } else if (angle < -M_PI) {
      while (angle < -M_PI) {
        angle += 2*M_PI;
      }
    }
    return angle;
  }
};
} /* namespace dynahead2_control */
