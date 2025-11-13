#include "behaviortree_ros/bt_service_node.h"
#include <std_srvs/SetBool.h>
class MyServiceCallerNode : public BT::RosServiceNode<std_srvs::SetBool>{
public:
  MyServiceCallerNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfiguration & conf):
  BT::RosServiceNode<std_srvs::SetBool>(nh, name, conf) {}

  static BT::PortsList providedPorts()
  {
    const char*  description = "Value to set";
    return {
      BT::InputPort<bool>("set_value", description)
    };
  }

  void sendRequest(std_srvs::SetBool::Request& request) override
  {
    auto input_data = getInput<bool>("set_value");
    if (!input_data) {
      throw BT::RuntimeError("missing required input [set_value]");
    }
    request.data = input_data.value();
  }

  BT::NodeStatus onResponse(const std_srvs::SetBool::Response& response) override
  {
    if (response.success)
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

