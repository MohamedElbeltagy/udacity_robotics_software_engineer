#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
 
using namespace std;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int counter = 0;
  bool run = true;

  while(run){
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Switch between goals
    switch (counter)
    {
    case 0:
      ROS_INFO("Going to pickup goal");
      goal.target_pose.pose.position.x = -2.0;
      goal.target_pose.pose.position.y = 1.0;
      goal.target_pose.pose.orientation.w = 1.0;
      break;
    case 1:
      ROS_INFO("Going to drop off goal");
      goal.target_pose.pose.position.x = 0.0;
      goal.target_pose.pose.position.y = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      break;
    }
    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      if(counter == 0) {
        ROS_INFO("Robot reached destination pick-up goal");
      }
      if(counter == 1){ 
        ROS_INFO("Robot reached destination drop-off goal");
      }
    }
    else
      ROS_INFO("Robot could not reached destination");

    // Incrementing goal points
    counter++; 
    // Delay 5 sec between each goal
    ros::Duration(5).sleep();

    // If reach the final goal, break the loop and exit
    if (counter == 2){
      run = false;
    } 
  }

  return 0;
}