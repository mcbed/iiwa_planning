#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/macros/console_colors.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <boost/algorithm/string.hpp>
#include <fstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("iiwa_mvif");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("iiwa_mvif", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "iiwa_arm";//"iiwa_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  
  std::vector<geometry_msgs::msg::Pose> waypoints ;
  RCLCPP_INFO(LOGGER, "loaded %li waypoints",waypoints.size());

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  moveit_msgs::msg::RobotTrajectory plan_trajectory, exec_trajectory;
  moveit_msgs::msg::RobotState robot_state;
  const double jump_threshold = 0.0;
  const double eef_step = 0.002;
  move_group.setPoseReferenceFrame(move_group.getPlanningFrame());
  RCLCPP_INFO(LOGGER, "pose reference frame set to %s",move_group.getPoseReferenceFrame().c_str());

  trajectory_processing::TimeOptimalTrajectoryGeneration traj_processor;
  robot_trajectory::RobotTrajectory traj(move_group.getRobotModel() ,PLANNING_GROUP);
  robot_trajectory::RobotTrajectory traj_plan(move_group.getRobotModel() ,PLANNING_GROUP);

  auto start_state = *move_group.getCurrentState();

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, plan_trajectory);
  RCLCPP_INFO(LOGGER, "Cartesian path : %.2f%% acheived", fraction * 100.0);
  traj.setRobotTrajectoryMsg(start_state,plan_trajectory);

  // for(auto i=0ul;i<waypoints.size();i++){
  //   move_group.setPoseTarget(waypoints[i],"tool0");
  //   // if(i>3){
  //   //   auto cm = createLineConstraint(waypoints[i-1],waypoints[i], visual_tools, move_group_node);
  //   //   move_group.setPathConstraints(cm);
  //   // }
  //   bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //   if (success){
  //     start_state = *move_group.getCurrentState();
  //     traj_plan.setRobotTrajectoryMsg(start_state,plan.trajectory_);
  //     traj.append(traj_plan,0.001);
  //     move_group.setStartState(traj.getLastWayPoint());
  //   }
  // }
  // traj_processor.computeTimeStamps(traj,0.2,0.01);
  for(auto i=0ul; i<traj.getWayPointDurations ().size();i++){
    // std::cout<< traj.getWayPointDurationFromPrevious(i) <<std::endl;
    if(traj.getWayPointDurationFromPrevious(i) == 0)
      traj.setWayPointDurationFromPrevious(i,0.0001);
  }
  traj.getRobotTrajectoryMsg(exec_trajectory);
  move_group.execute(exec_trajectory);

  rclcpp::shutdown();
  return 0;
}
