#include <iostream>
#include <vector>
#include <algorithm>
#include "rebet_msgs/srv/detect_object.hpp"
#include "rebet_msgs/msg/objects_identified.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using DetectObject = rebet_msgs::srv::DetectObject;
using ObjectsIdentified = rebet_msgs::msg::ObjectsIdentified;

class DetectObjectService : public RosServiceNode<DetectObject>
{
public:
    static constexpr const char* OBJS_DETECTED= "objs_detected";

    DetectObjectService(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<DetectObject>(instance_name, conf, params)
    {
        all_object_ids = {};
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosServiceNode::providedPorts();

        PortsList child_ports = { 
                OutputPort<std::vector<ObjectsIdentified>>(OBJS_DETECTED)
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(logger(), "ID object request");

        setOutput("name_of_task",registrationName());
        request->id = 1; //fix
    
        return true;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "response received");

        ObjectsIdentified obj_idd = response.get()->objects_id;

        all_object_ids.push_back(obj_idd);

        setOutput(OBJS_DETECTED, all_object_ids);

        // RCLCPP_INFO(logger(), ss.str().c_str());
        RCLCPP_INFO(logger(), "SUCCESS IN IDOBJ");
        return NodeStatus::SUCCESS;
    }

    private:
        std::vector<ObjectsIdentified> all_object_ids;
};

BT_REGISTER_ROS_NODES(factory, params)
{
    RosNodeParams aug_params;
    aug_params.nh = params.nh;
    aug_params.server_timeout = std::chrono::milliseconds(40000); //The YOLO can take quite a while.

    factory.registerNodeType<DetectObjectService>("DetectObject", aug_params);
}

}