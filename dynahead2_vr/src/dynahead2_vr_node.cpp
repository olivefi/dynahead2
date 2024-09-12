#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <dynahead2_vr/Dynahead2VR.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<dynahead2_vr::Dynahead2VR> node(argc, argv, "dynahead2_vr", 1);
  return static_cast<int>(!node.execute());
}


