#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal_to_move_base(MoveBaseClient & ac, double x, double y, double orient)
{
  move_base_msgs::MoveBaseGoal goal;

  // Set up frame and stamp parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = orient;

  // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("Failed to reach goal for some reason.");
}

int main(int argc, char** argv){

    // Define pick-up & drop-off locations
    double pick_up_position[2]  = {0.0, 3.0};
    double drop_off_position[2] = {7.0, 4.0};


    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    ROS_INFO("Initiallizing navigation to pick-up & drop-off zones.."); 

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait 5s for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Send the goal position and orientation for the robot to reach 
    ROS_INFO("Sending pick-up goal..");   
    send_goal_to_move_base(ac, pick_up_position[0], pick_up_position[1], 1.0);
    
    // Simulate the picking up
    sleep(5); 
    
    // Send the goal position and orientation for the robot to reach 
    ROS_INFO("Sending drop-off goal..");
    send_goal_to_move_base(ac, drop_off_position[0], drop_off_position[1], 1.0);

    return 0;
}