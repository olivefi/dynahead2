#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include <dynahead2_control/Dynahead2Control.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<dynahead2_control::Dynahead2Control> node(argc, argv, "dynahead2_control", 1);
  return static_cast<int>(!node.execute());
}


