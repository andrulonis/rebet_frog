#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using Odometry = nav_msgs::msg::Odometry;


class RobotPoseNode : public RosTopicSubNode<Odometry>
{
    public:
    
    static constexpr const char* POSE_OUT = "out_pose";

    RobotPoseNode(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicSubNode<Odometry>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicSubNode::providedPorts();

        PortsList child_ports = {
                        OutputPort<Odometry>(POSE_OUT),
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received since the last tick.
   * it might be empty.
   * @return the new status of the Node, based on last_msg
   */
    BT::NodeStatus onTick(const typename Odometry::SharedPtr& last_msg) override
    {
        std::cout << "Inside robot_pose_sub " << std::endl;
        if(last_msg) {
            setOutput(POSE_OUT, *last_msg);
            return BT::NodeStatus::SUCCESS;
            

        }
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(RobotPoseNode, "getRobotPose");

}