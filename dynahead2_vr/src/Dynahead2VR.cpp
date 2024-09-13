#include "dynahead2_vr/Dynahead2VR.hpp"

namespace dynahead2_vr {

Dynahead2VR::Dynahead2VR(any_node::Node::NodeHandlePtr nh)
    : any_node::Node(nh) {}

bool Dynahead2VR::init() {
  updateRate_ = param<double>("update_rate", 100);

  dynahead2JointStateSub_ = nh_->subscribe("/dynahead2_control/joint_states", 1, &Dynahead2VR::dynahead2JointStateCallback, this);
  vrStateSub_ = nh_->subscribe("/quest/pose/headset", 1, &Dynahead2VR::vrStateCallback, this);
  dynahead2JointCmdPub_ = nh_->advertise<sensor_msgs::JointState>("/dynahead2_control/joint_commands", 1);

  updateRate_ = param<double>("update_rate", 20);
  vrScaling_ = param<double>("vr_scaling", 1.0);

  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&Dynahead2VR::update, this, _1);
  workerOptions.timeStep_ = 1.0 / updateRate_;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void Dynahead2VR::cleanup() {
  dynahead2JointCmdPub_.shutdown();
  dynahead2JointStateSub_.shutdown();
  vrStateSub_.shutdown();
}

bool Dynahead2VR::update(const any_worker::WorkerEvent &event) {
  PitchYaw vrRelativeAngles;
  vrRelativeAngles.pitch = vrAngles_.pitch - vrAnglesInit_.pitch;
  vrRelativeAngles.yaw = vrAngles_.yaw - vrAnglesInit_.yaw;

  if (!dynahead2InitReceived_ || !vrInitReceived_) {
    return true;
  }
  sensor_msgs::JointState dynahead2JointCmd;
  dynahead2JointCmd.header.stamp = ros::Time::now();
  dynahead2JointCmd.name.push_back("pitch");
  dynahead2JointCmd.name.push_back("yaw");
  dynahead2JointCmd.position.push_back(dynahead2AnglesInit_.pitch + vrRelativeAngles.pitch * vrScaling_);
  dynahead2JointCmd.position.push_back(dynahead2AnglesInit_.yaw + vrRelativeAngles.yaw * vrScaling_);
  dynahead2JointCmdPub_.publish(dynahead2JointCmd);

  return true;
}

void Dynahead2VR::vrStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  Eigen::Matrix3d rotM = q.toRotationMatrix();
  vrAngles_.pitch = atan2(-rotM(2,0), sqrt(pow(rotM(1,0),2)+pow(rotM(2,2),2)));
  vrAngles_.yaw = atan2(rotM(1,0), rotM(0,0));
  if (!vrInitReceived_) {
    vrAnglesInit_ = vrAngles_;
    vrInitReceived_ = true;
  }
}

void Dynahead2VR::dynahead2JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  std::map<std::string, double> dynahead2JointMap;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    dynahead2JointMap[msg->name[i]] = msg->position[i];
  }
  dynahead2Angles_.pitch = dynahead2JointMap["pitch"];
  dynahead2Angles_.yaw = dynahead2JointMap["yaw"];
  if (!dynahead2InitReceived_) {
    dynahead2AnglesInit_ = dynahead2Angles_;
    dynahead2InitReceived_ = true;
  }
}

} /* namespace dynahead2_vr */
