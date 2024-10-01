#include "vr_teleop/VRTeleop.hpp"

namespace vr_teleop {

VRTeleop::VRTeleop(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool VRTeleop::init() {
  updateRate_ = param<double>("update_rate", 100);
  joyTimeout_ = param<double>("timeout", 0.1);
  xScaling_ = param<double>("x_scaling", 1.0);
  yScaling_ = param<double>("y_scaling", 1.0);
  zScaling_ = param<double>("z_scaling", 1.0);

  twistPub_ = nh_->advertise<geometry_msgs::TwistStamped>("/teleop/base/twist", 1);
  vrJoySub_ = nh_->subscribe("/quest/joystick", 1, &VRTeleop::vrJoyCallback, this);
  // imu sub is used for time sync
  imuSub_ = nh_->subscribe("/sensors/imu", 1, &VRTeleop::imuCallback, this);

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&VRTeleop::update, this, _1);
  workerOptions.timeStep_ = 1.0 / updateRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void VRTeleop::cleanup() {
  twistPub_.shutdown();
  vrJoySub_.shutdown();
  imuSub_.shutdown();
}

bool VRTeleop::update(const any_worker::WorkerEvent &event) {
  geometry_msgs::TwistStamped twist = twist_;
  twist.header.stamp = followerDeviceTime_;
  if ((ros::Time::now() - lastJoyReceived_).toSec() > joyTimeout_) {
    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = 0.0;
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    ROS_WARN_STREAM_THROTTLE(1, "[VR Teleop] Joystick readings too old! Sending zero twist.");
  }
  twistPub_.publish(twist);
  return true;
}

void VRTeleop::vrJoyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  if (msg->axes.size() < 4) {
    ROS_ERROR_STREAM_THROTTLE(1, "[VR Teleop] Received joy message with less than 4 axes");
    return;
  }
  twist_.twist.linear.x = msg->axes[JOY_RIGHT_X_AXIS] * xScaling_;
  twist_.twist.linear.y = msg->axes[JOY_RIGHT_Y_AXIS] * yScaling_;
  twist_.twist.angular.z = msg->axes[JOY_LEFT_Y_AXIS] * zScaling_;
  lastJoyReceived_ = ros::Time::now();
}

void VRTeleop::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  followerDeviceTime_ = msg->header.stamp;
}

} /* namespace vr_teleop */
