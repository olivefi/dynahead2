#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <ros/ros.h>

#include <any_node/any_node.hpp>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

namespace dynahead2_vr {

struct PitchYaw {
  double pitch;
  double yaw;
};

class Dynahead2VR : public any_node::Node {
public:
  Dynahead2VR(any_node::Node::NodeHandlePtr nh);
  ~Dynahead2VR() override = default;

  bool init() override;
  void cleanup() override;
  bool update(const any_worker::WorkerEvent &event);

protected:
  // Methods

  // Externally settable variables
  double updateRate_;
  double vrScaling_;

  // Internal state variables
  PitchYaw dynahead2AnglesInit_;
  PitchYaw vrAnglesInit_;
  PitchYaw dynahead2Angles_;
  PitchYaw vrAngles_;

  bool dynahead2InitReceived_;
  bool vrInitReceived_;

  // Callbacks
  void dynahead2JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void vrStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

  // Publishers + Subscribers
  ros::Publisher dynahead2JointCmdPub_;
  ros::Subscriber dynahead2JointStateSub_;
  ros::Subscriber vrStateSub_;
};
} /* namespace dynahead2_vr */
