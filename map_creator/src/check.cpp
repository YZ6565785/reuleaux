#include <ros/ros.h>
#include <iostream>

// moveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_check");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
  current_state.setVariablePositions(std::vector<double>{0.0, 1.03, 0.0, 0.0, 0.0, 0.0});
  current_state.printStatePositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  current_state.setVariablePositions(std::vector<double>{0.0, 0.0, 0.19, 0.0, 0.0, 0.0});
  current_state.printStatePositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  std::cout << "This is collision check node\t DONE." << std::endl;
  return 0;
}