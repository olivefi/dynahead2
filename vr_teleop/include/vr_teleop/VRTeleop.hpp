#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Time.h>

namespace vr_teleop {

// these are the indices of the respective axes in the joy message
const unsigned int JOY_LEFT_X_AXIS = 3;
const unsigned int JOY_LEFT_Y_AXIS = 2;
const unsigned int JOY_RIGHT_X_AXIS = 1;
const unsigned int JOY_RIGHT_Y_AXIS = 0;

class VRTeleop : public any_node::Node {
public:
  VRTeleop(any_node::Node::NodeHandlePtr nh);
  ~VRTeleop() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent &event);

protected:
  // Methods

  // Externally settable variables
  double updateRate_;
  double joyTimeout_;
  double xScaling_;
  double yScaling_;
  double zScaling_;

  // Internal variables
  ros::Time followerDeviceTime_;
  ros::Time lastJoyReceived_;
  geometry_msgs::TwistStamped twist_;

  // Callbacks
  void vrJoyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void followerTimeCallback(const std_msgs::Time::ConstPtr &msg);

  // Publishers + Subscribers
  ros::Publisher twistPub_;
  ros::Subscriber vrJoySub_;
  ros::Subscriber followerTimeSub_;
};
} /* namespace vr_teleop */
