#include "rclcpp/rclcpp.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/macros/console_colors.h>

#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_response.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>

#include "helper_tools.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_cartesian_pilz");
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const auto node = rclcpp::Node::make_shared("robot_cartesian_pilz", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools rvisual_tools("world", "path", node);
    rvisual_tools.deleteAllMarkers();

    const std::string PLANNING_GROUP = "iiwa_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(node,"robot_description");
    const moveit::core::RobotModelConstPtr & robot_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);  

      std::string waypoint_file_path, waypoint_file_type;
    std::vector<double> shift;
    double scale;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    node->get_parameter("waypoint_file_path", waypoint_file_path);
    node->get_parameter("waypoint_file_type", waypoint_file_type);
    node->get_parameter("shift", shift);
    node->get_parameter("scale", scale);
    RCLCPP_INFO(LOGGER, "Loading waypoints from path: %s", waypoint_file_path.c_str());
    if(waypoint_file_type == "csv")
        waypoints = csv2path(waypoint_file_path, shift, scale);
    else if(waypoint_file_type == "apt")
        waypoints = apt2path(waypoint_file_path, shift, scale);

    RCLCPP_INFO(LOGGER, "loaded %li waypoints",waypoints.size());

    bool plot_waypoints;
    bool plot_frames;
    node->get_parameter("plot_waypoints", plot_waypoints);
    node->get_parameter("plot_frames", plot_frames);

    // // Visualize the plan in RViz
    rvisual_tools.deleteAllMarkers();
    rvisual_tools.setBaseFrame("world");
    if(plot_waypoints){
        rvisual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XXXXSMALL);
    }
    if(plot_frames){
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        rvisual_tools.publishAxisLabeled(waypoints[i], "", rvt::XXXXSMALL);
    }
    rvisual_tools.trigger();

    //Setup
    std::vector<double> tolerance_pose(3, 1e-8);
    std::vector<double> tolerance_angle(3, 1e-8);
    
    moveit::planning_interface::MoveGroupInterface::Plan planned_trajectory;
    moveit_msgs::msg::MotionSequenceRequest sequenceRequest;
    moveit_msgs::msg::MotionSequenceResponse sequenceResponse;
    moveit_msgs::msg::MotionSequenceItem motionItem;
    moveit_msgs::msg::Constraints pose_goal;
    planning_interface::MotionPlanRequest req;
    geometry_msgs::msg::PoseStamped pose;


    double velcocity_scaling, acceleration_scaling;
    node->get_parameter("velcocity_scaling", velcocity_scaling);
    node->get_parameter("acceleration_scaling", acceleration_scaling);

    req.planner_id = "LIN";
    req.group_name = PLANNING_GROUP;
    req.max_velocity_scaling_factor = velcocity_scaling;
    req.max_acceleration_scaling_factor = acceleration_scaling;
    req.allowed_planning_time = 10.0;

    for(auto w = 0ul; w < waypoints.size(); w++){
        pose = move_group.getCurrentPose("tool0");
        pose.pose = waypoints[w];
        pose_goal = kinematic_constraints::constructGoalConstraints(
            "tool0", pose, tolerance_pose, tolerance_angle
        );
        req.goal_constraints.clear();
        req.goal_constraints.push_back(pose_goal);
        req.start_state = moveit_msgs::msg::RobotState();

        motionItem.set__req(req);
        motionItem.set__blend_radius(0.0);

        sequenceRequest.items.push_back(motionItem);
    }

    // Plan trough service
    rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedPtr client =
        node->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");

    auto request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
    request->set__request(sequenceRequest);

    while (!client->wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
            "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    sequenceResponse = result.get()->response;

    planned_trajectory.trajectory_ = sequenceResponse.planned_trajectories.at(0);
    planned_trajectory.planning_time_ = sequenceResponse.planning_time;
    planned_trajectory.start_state_ = sequenceResponse.sequence_start;

    // Execute plan
    move_group.execute(planned_trajectory);

    rclcpp::shutdown();

    return 0;
}
