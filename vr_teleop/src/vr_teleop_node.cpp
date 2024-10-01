#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <vr_teleop/VRTeleop.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<vr_teleop::VRTeleop> node(argc, argv, "vr_teleop", 1);
  return static_cast<int>(!node.execute());
}


