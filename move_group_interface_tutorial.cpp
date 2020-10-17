/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */



/*********************************************************************
 * 용태's COMMENT
 * move_group.plan : planning
 * move_group.excute : execute
 * move_group.move : planning & execute -> can be different with planning above
 *********************************************************************/







#define _USE_MATH_DEFINES
#include <cmath>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double x_orient = 0.4;
  double y_orient = -0.05;
  double z_orient = 0.225;

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("dummy_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");














//
//
//
//  ///////////////////////////////////////////////////////////////////////////////////////
//  ///////////////////////// Planning to a joint-space goal 1 ////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////////////
//  //
//  // Let's set a joint space goal and move towards it.  This will replace the
//  // pose target we set above.
//  //
//  // To start, we'll create an pointer that references the current robot's state.
//  // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions;
//  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  joint_group_positions[0] = 0.74;  // radians
//  joint_group_positions[1] = 0.87;  // radians
//  joint_group_positions[2] = -0.35;  // radians
//  joint_group_positions[3] = -0.43;  // radians
//  joint_group_positions[4] = 0.3;  // radians
//  joint_group_positions[5] = 0.91;  // radians
//  joint_group_positions[6] = 0.67;  // radians
//
//  move_group.setJointValueTarget(joint_group_positions);
//
//  // Now, we call the planner to compute the plan and visualize it.
//  // Note that we are just planning, not asking move_group
//  // to actually move the robot.
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
//  bool success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan1);
//  /////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////////
//  move_group.setStartStateToCurrentState();
//
//
//
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
////  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//
//
//
//
//
//
//
//
//  ///////////////////////////////////////////////////////////////////////////////////////
//  ///////////////////////// Planning to a joint-space goal 2 ////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////////////
//  //
//  // Let's set a joint space goal and move towards it.  This will replace the
//  // pose target we set above.
//  //
//  // To start, we'll create an pointer that references the current robot's state.
//  // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state2 = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_group_positions2;
//  current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions2);
//
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  joint_group_positions2[0] = 1.3;  // radians
//  joint_group_positions2[1] = -0.8;  // radians
//  joint_group_positions2[2] = -0.97;  // radians
//  joint_group_positions2[3] = -0.5;  // radians
//  joint_group_positions2[4] = -1;  // radians
//  joint_group_positions2[5] = -0.6;  // radians
//  joint_group_positions2[6] = -0.16;  // radians
//
//  move_group.setJointValueTarget(joint_group_positions2);
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
//  success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan2);
//  /////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////////
//  move_group.setStartStateToCurrentState();
//
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
////  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
//
//
//
//
//


  /////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Planning to a Pose goal  /////////////////////////////
  ///////////////////////////// Pick & Place Test Pose ///////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  // Euler(roll,pitch,yaw)값을 quarternion 으로 바꾸기
  // Quarternion 값을 q2 라고 내가 정해준다. 함수는 setEulerZYX 이용.
  tf::Quaternion q;
  q.setEulerZYX(0,0,-2*M_PI/3);
  // 이 값을 orientation 에 이용.

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  target_pose1.position.x = x_orient;
  target_pose1.position.y = y_orient;
  target_pose1.position.z = z_orient;
  move_group.setPoseTarget(target_pose1);

  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
  move_group.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  bool success = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan3);
  /////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////////
  move_group.setStartStateToCurrentState();


  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");






//  //////////////////////////////////////////////
//  ///////////////////////////// Planning to a Pose goal  /////////////////////////////
//  ///////////////////////////// Repeatability Test Pose ///////////////////////////////
//  /////////////////////////////////////////////////////////////////////////////////////
//  // We can plan a motion for this group to a desired pose for the
//  // end-effector.
//
//  // Euler(roll,pitch,yaw)값을 quarternion 으로 바꾸기
//  // Quarternion 값을 q2 라고 내가 정해준다. 함수는 setEulerZYX 이용.
//  tf::Quaternion q;
//  q.setEulerZYX(0,0,M_PI/2);
//  // 이 값을 orientation 에 이용.
//
//  geometry_msgs::Pose target_pose1;
//  target_pose1.orientation.x = q.x();
//  target_pose1.orientation.y = q.y();
//  target_pose1.orientation.z = q.z();
//  target_pose1.orientation.w = q.w();
//  target_pose1.position.x = x_orient;
//  target_pose1.position.y = y_orient;
//  target_pose1.position.z = z_orient;
//  move_group.setPoseTarget(target_pose1);
////  move_group.setPoseTarget([0.25, -0.25, 0.2, 3.14, 0, 0]);
//
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
//  success = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan3);
//  /////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////////
//  move_group.setStartStateToCurrentState();
//
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
//
//  // Visualizing plans
//  // ^^^^^^^^^^^^^^^^^
//  // We can also visualize the plan as a line with markers in RViz.
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//  visual_tools.publishAxisLabeled(target_pose1, "pose1");
//  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
////  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue t
//
//
//
//
//
//
//
//
//
  /////////////////////////////////////////////////////////////////////////////////////
  //////////////////////// Planning with Path Constraints /////////////////////////////
  ////////////////////////         Push the button        /////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link7";
  ocm.header.frame_id = "dummy_link";


  ocm.orientation.x = q.x();
  ocm.orientation.y = q.y();
  ocm.orientation.z = q.z();
  ocm.orientation.w = q.w();


  ocm.absolute_x_axis_tolerance = 0.05;
  ocm.absolute_y_axis_tolerance = 0.05;
  ocm.absolute_z_axis_tolerance = 0.05;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  // EEF의 orientation에 대한 constraint 준 것.
  // 각 축에 대한 tolerance & weight(이건 잘 모르겠음)
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  //   Note that this will only work if the current state already satisfies the path constraints.
  //   So, we need to set the start state to a new pose.

//  robot_state::RobotState start_state(*move_group.getCurrentState());
//  geometry_msgs::Pose start_pose2;
//
//  /////////// q = tf::Quaternion q; 로 위에 ocm에서 정의 해줌  //////////
//  //////////  q =  q.setEulerZYX(0,-M_PI,0);    //////////
//  start_pose2.orientation.x = q.x();
//  start_pose2.orientation.y = q.y();
//  start_pose2.orientation.z = q.z();
//  start_pose2.orientation.w = q.w();
//
//  start_pose2.position.x = 0.3;
//  start_pose2.position.y = -0.3;
//  start_pose2.position.z = 0.10;
//  start_state.setFromIK(joint_model_group, start_pose2);
//  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new start state that we have just created.
  // 처음에 정해준 End effector의 pose = target_pose1
  //////////////// Target Pose 변경!!! //////////////////////////
  //////////////// 밑으로 누르는거로 //////////////////////////////
  geometry_msgs::Pose target_pose2;

  /////////// q = tf::Quaternion q; 로 위에 ocm에서 정의 해줌  //////////
  //////////  q =  q.setEulerZYX(0,-M_PI,0);    //////////
  target_pose2.orientation.x = q.x();
  target_pose2.orientation.y = q.y();
  target_pose2.orientation.z = q.z();
  target_pose2.orientation.w = q.w();

  target_pose2.position.x = x_orient;
  target_pose2.position.y = y_orient + 0.15 ;
  target_pose2.position.z = z_orient ;
//  start_state.setFromIK(joint_model_group, start_pose2);

  move_group.setPoseTarget(target_pose2);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(20.0);

  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
  move_group.setMaxVelocityScalingFactor(0.7);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
  success = (move_group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan4);


  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
//  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

//  // When done with the path constraint be sure to clear it.
//////////////////////////////////////////////////////////////////////////////////////////
////////////////// 다시 원래대로 돌아오게 만들 때까지는 PATH CONSTRAINT 유지!!! //////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  ////////////// path constraint의 마지막(target pose, current state)을 다음 모션의 start state으로! ////////////
  move_group.setStartStateToCurrentState();













  /////////////////////////////////////////////////////////////////////////////////////
  //////////////////////// Planning with Path Constraints /////////////////////////////
  ////////////////////////      Go back to Position       /////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  //


  // Now we will plan to the earlier pose target from the new start state that we have just created.
  // 처음에 정해준 End effector의 pose = target_pose1
  //////////////// Target Pose 변경!!! //////////////////////////
  //////////////// 밑으로 누르는거로 //////////////////////////////
  geometry_msgs::Pose target_pose3;
  target_pose3.orientation.x = q.x();
  target_pose3.orientation.y = q.y();
  target_pose3.orientation.z = q.z();
  target_pose3.orientation.w = q.w();

  target_pose3.position.x = x_orient;
  target_pose3.position.y = y_orient;
  target_pose3.position.z = z_orient;
//  start_state.setFromIK(joint_model_group, start_pose2);


  move_group.setPoseTarget(target_pose3);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(20.0);

  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
//  move_group.setMaxVelocityScalingFactor(0.5);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
  success = (move_group.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan5);


  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
//  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  ////////////// path constraint의 마지막(target pose, current state)을 다음 모션의 start state으로! ////////////
  move_group.setStartStateToCurrentState();


















/////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Planning to a Pose goal  /////////////////////////////
  ///////////////////////////// Place Pose ///////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  // Euler(roll,pitch,yaw)값을 quarternion 으로 바꾸기
  // Quarternion 값을 q2 라고 내가 정해준다. 함수는 setEulerZYX 이용.
  tf::Quaternion q2;
  q2.setEulerZYX(0,0,M_PI);
  // 이 값을 orientation 에 이용.

  geometry_msgs::Pose target_pose10;
  target_pose10.orientation.x = q2.x();
  target_pose10.orientation.y = q2.y();
  target_pose10.orientation.z = q2.z();
  target_pose10.orientation.w = q2.w();
  target_pose10.position.x = 0;
  target_pose10.position.y = -0.7;
  target_pose10.position.z = 0.1;
  move_group.setPoseTarget(target_pose10);

  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
  move_group.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan1);
  /////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////////
  move_group.setStartStateToCurrentState();


  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");














  ////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////// Go to HOME POSITION ///////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state5 = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_home_positions;
  current_state5->copyJointGroupPositions(joint_model_group, joint_home_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_home_positions[0] = 0;  // radians
  joint_home_positions[1] = 0;  // radians
  joint_home_positions[2] = 0;  // radians
  joint_home_positions[3] = 0;  // radians
  joint_home_positions[4] = 0;  // radians
  joint_home_positions[5] = 0;  // radians
  joint_home_positions[6] = 0;  // radians


  move_group.setJointValueTarget(joint_home_positions);
  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
  move_group.setMaxVelocityScalingFactor(1.0);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
  success = (move_group.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan6);


  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");





  /////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////    반복 반복 반복하기   //////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////

for (int i = 0 ; i < 15 ; i++)
{


  move_group.execute(my_plan3);
  sleep(1.0);

  // PLAN 4,5 는 이동속도 느리게 //
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.setMaxVelocityScalingFactor(0.7);
  move_group.execute(my_plan4);
  sleep(1.0);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  move_group.execute(my_plan5);
  sleep(1.0);



  // 이동속도 원위치 //
  move_group.setMaxVelocityScalingFactor(1.0);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.execute(my_plan1);
  sleep(1.0);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  move_group.execute(my_plan6);
  sleep(1.0);

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}


//
//for (int i = 0 ; i < 20 ; i++)
//{
//
//  move_group.execute(my_plan1);
//  sleep(2.0);
//  move_group.execute(my_plan2);
//  sleep(2.0);
//  move_group.execute(my_plan3);
//  sleep(2.0);
//
//  // PLAN 4,5 는 이동속도 느리게 //
//  move_group.setMaxVelocityScalingFactor(0.5);
//  move_group.execute(my_plan4);
//  sleep(10.0);
//  move_group.execute(my_plan5);
//  sleep(2.0);
//
//  // 이동속도 원위치 //
//  move_group.setMaxVelocityScalingFactor(1.0);
//  move_group.execute(my_plan6);
//  sleep(2.0);
//
//}
//
//
//for (int i = 0 ; i < 20 ; i++)
//{
//
//  move_group.execute(my_plan1);
//  sleep(2.0);
//  move_group.execute(my_plan2);
//  sleep(2.0);
//  move_group.execute(my_plan3);
//  sleep(2.0);
//
//  // PLAN 4,5 는 이동속도 느리게 //
//  move_group.setMaxVelocityScalingFactor(0.8);
//  move_group.execute(my_plan4);
//  sleep(10.0);
//  move_group.execute(my_plan5);
//  sleep(2.0);
//
//  // 이동속도 원위치 //
//  move_group.setMaxVelocityScalingFactor(1.0);
//  move_group.execute(my_plan6);
//  sleep(2.0);
//
//}






//  /////////////////////////////////////////////////////////////////////////////////////
//  /////////////////////////////// Cartesian Paths /////////////////////////////////////
//  /////////////////////////////////////////////////////////////////////////////////////
//  // You can plan a Cartesian path directly by specifying a list of waypoints
//  // for the end-effector to go through. Note that we are starting
//  // from the new start state above.  The initial pose (start state) does not
//  // need to be added to the waypoint list but adding it can help with visualizations
//  geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;
//
//  std::vector<geometry_msgs::Pose> waypoints;
//  waypoints.push_back(target_pose3);
//
//  target_pose3.position.z += 0.15;
//  waypoints.push_back(target_pose3);  // down
//
//  target_pose3.position.x -= 0.1;
//  waypoints.push_back(target_pose3);  // right
//
//  target_pose3.position.z += 0.05;
//  target_pose3.position.y += 0.05;
////  target_pose3.position.x -= 0.1;
//  waypoints.push_back(target_pose3);  // up and left
//
//  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
//  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
//  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
//  move_group.setMaxVelocityScalingFactor(0.2);
//
//  // We want the Cartesian path to be interpolated at a resolution of 1 cm
//  // which is why we will specify 0.01 as the max step in Cartesian
//  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
//  // Warning - disabling the jump threshold while operating real hardware can cause
//  // large unpredictable motions of redundant joints and could be a safety issue
//  moveit_msgs::RobotTrajectory trajectory;
//  const double jump_threshold = 0.0;
//  const double eef_step = 0.01;
//  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//  /// computeCartesianPath -> trajectory가 얼마만큼 cartesian path를 따라갔는 지를 return 함! ///
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
//
//
////  move_group.plan(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
////  move_group.execute(trajectory);
//
//  //// move_group.plan(my_plan) 과 같은 게 없어서 Cartesian path 를 진행했어도 /////////
//  //// 계속해서 path constraint의 target pose가 start state로 정의되어있음.    /////////
//
//
//
//  // Since we set the start state we have to clear it before planning other paths
//  //////////// Cartesian path의 마지막(current state)을 다음 모션의 start state으로! ///////////
//  move_group.setStartStateToCurrentState();
//
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//  for (std::size_t i = 0; i < waypoints.size(); ++i)
//    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");











//  // Adding/Removing Objects and Attaching/Detaching Objects
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // Define a collision object ROS message.
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = move_group.getPlanningFrame();
//
//  // The id of the object is used to identify it.
//  collision_object.id = "box1";
//
//  // Define a box to add to the world.
//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = primitive.BOX;
//  primitive.dimensions.resize(3);
//  primitive.dimensions[0] = 1.5;
//  primitive.dimensions[1] = 0.1;
//  primitive.dimensions[2] = 0.5;
//
//  // Define a pose for the box (specified relative to frame_id)
//  // 원점의 좌표 (x,y,z) -> x_dimension = 0.5 라면 원점 좌표로 부터 양쪽으로 0.25씩 크기발생
//  geometry_msgs::Pose box_pose;
//  box_pose.orientation.w = 1.0;
//  box_pose.position.x = 0.0;
//  box_pose.position.y = 0.4;
//  box_pose.position.z = 0.85;
//
//  collision_object.primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose);
//  collision_object.operation = collision_object.ADD;
//
//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.push_back(collision_object);
//
//  // Now, let's add the collision object into the world
//  ROS_INFO_NAMED("tutorial", "Add an object into the world");
//  planning_scene_interface.addCollisionObjects(collision_objects);
//
//
//
//  // Define a collision object ROS message.
//  moveit_msgs::CollisionObject collision_object2;
//  collision_object2.header.frame_id = move_group.getPlanningFrame();
//
//  // The id of the object is used to identify it.
//  collision_object2.id = "box2";
//
//  // Define a box to add to the world.
//  shape_msgs::SolidPrimitive primitive2;
//  primitive2.type = primitive2.BOX;
//  primitive2.dimensions.resize(3);
//  primitive2.dimensions[0] = 1.2;
//  primitive2.dimensions[1] = 0.1;
//  primitive2.dimensions[2] = 0.5;
//
//  // Define a pose for the box (specified relative to frame_id)
//  // 원점의 좌표 (x,y,z) -> x_dimension = 0.5 라면 원점 좌표로 부터 양쪽으로 0.25씩 크기발생
//  geometry_msgs::Pose box_pose2;
//  box_pose2.orientation.w = 1.0;
//  box_pose2.position.x = -0.15;
//  box_pose2.position.y = 0.4;
//  box_pose2.position.z = 0.35;
//
//  collision_object2.primitives.push_back(primitive2);
//  collision_object2.primitive_poses.push_back(box_pose2);
//  collision_object2.operation = collision_object2.ADD;
//
//  std::vector<moveit_msgs::CollisionObject> collision_objects2;
//  collision_objects2.push_back(collision_object2);
//
//  // Now, let's add the collision object into the world
//  ROS_INFO_NAMED("tutorial", "Add an object2 into the world");
//  planning_scene_interface.addCollisionObjects(collision_objects2);
//
//
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  // Wait for MoveGroup to recieve and process the collision object message
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");






//
//  ////////////////////////////////////////////////////////////////////////////////////
//  /////////// Now when we plan a trajectory it will avoid the obstacle ///////////////
//  ////////////////////////////////////////////////////////////////////////////////////
//  move_group.setStartState(*move_group.getCurrentState());
//
//
//  // Euler(roll,pitch,yaw)값을 quarternion 으로 바꾸기
//  // Quarternion 값을 q2 라고 내가 정해준다. 함수는 setEulerZYX 이용.
//  tf::Quaternion q3;
//  q3.setEulerZYX(0,-M_PI,0);
//  // 이 값을 orientation 에 이용.
//
//  geometry_msgs::Pose another_pose;
//  another_pose.orientation.x = q3.x();
//  another_pose.orientation.y = q3.y();
//  another_pose.orientation.z = q3.z();
//  another_pose.orientation.w = q3.w();
//  another_pose.position.x = 0.3;
//  another_pose.position.y = 0.3;
//  another_pose.position.z = 0.2;
//  move_group.setPoseTarget(another_pose);
//
//
//
//
//  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
//  move_group.setMaxVelocityScalingFactor(0.5);
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan8;
//
//
//  success = (move_group.plan(my_plan8) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan8);
//
//  ////////////// Motion Planning의 마지막 state를 다음 모션의 start state으로! ///////////////
//  move_group.setStartStateToCurrentState();
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan8.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("next step");













//  // Now, let's attach the collision object to the robot.
//  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
//  move_group.attachObject(collision_object.id);
//  move_group.attachObject(collision_object2.id);
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
//                      "robot");
//
//  // Now, let's detach the collision object from the robot.
//  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
//  move_group.detachObject(collision_object.id);
//  move_group.detachObject(collision_object2.id);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
//                      "robot");
//
//  // Now, let's remove the collision object from the world.
//  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
//  std::vector<std::string> object_ids;
//  object_ids.push_back(collision_object.id);
//  planning_scene_interface.removeCollisionObjects(object_ids);
//
//  std::vector<std::string> object2_ids;
//  object2_ids.push_back(collision_object2.id);
//  planning_scene_interface.removeCollisionObjects(object2_ids);
//
//  // Show text in RViz of status
//  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  /* Wait for MoveGroup to recieve and process the attached collision object message */
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
//
//
//
//
//
//
//
//
//  ////////////////////////////////////////////////////////////////////////////////////
//  //////////////////////////// Go to HOME POSITION ///////////////////////////////////
//  ////////////////////////////////////////////////////////////////////////////////////
//  //
//  // Let's set a joint space goal and move towards it.  This will replace the
//  // pose target we set above.
//  //
//  // To start, we'll create an pointer that references the current robot's state.
//  // RobotState is the object that contains all the current position/velocity/acceleration data.
//  moveit::core::RobotStatePtr current_state5 = move_group.getCurrentState();
//  //
//  // Next get the current set of joint values for the group.
//  std::vector<double> joint_home_positions;
//  current_state5->copyJointGroupPositions(joint_model_group, joint_home_positions);
//
//  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
//  joint_home_positions[0] = 0;  // radians
//  joint_home_positions[1] = 0;  // radians
//  joint_home_positions[2] = 0;  // radians
//  joint_home_positions[3] = 0;  // radians
//  joint_home_positions[4] = 0;  // radians
//  joint_home_positions[5] = 0;  // radians
//  joint_home_positions[6] = 0;  // radians
//
//
//  move_group.setJointValueTarget(joint_home_positions);
//  /////////////// 최대속도의  --------% 로 움직여라 /////////////////////////
//  move_group.setMaxVelocityScalingFactor(1.0);
//
//
//  moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
//  success = (move_group.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  move_group.execute(my_plan6);
//
//
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in RViz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
////  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//  visual_tools.trigger();
//  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");










  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
