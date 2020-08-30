#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "pick_objects");

    MoveBaseClient ac("move_base", true);
    
    ros::NodeHandle node_handle;
    ros::Publisher pickup_publisher = node_handle.advertise<std_msgs::Bool>("/pickup", 10);
    ros::Publisher dropoff_publisher = node_handle.advertise<std_msgs::Bool>("/dropoff", 10);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server...");
    }

    move_base_msgs::MoveBaseGoal goal_pickup_zone;
    move_base_msgs::MoveBaseGoal goal_dropoff_zone;
    move_base_msgs::MoveBaseGoal goal_home;

    goal_pickup_zone.target_pose.header.frame_id = "map";
    goal_pickup_zone.target_pose.header.stamp = ros::Time::now();

    goal_pickup_zone.target_pose.pose.position.x = 3.61;
    goal_pickup_zone.target_pose.pose.position.y = -4.68;
    goal_pickup_zone.target_pose.pose.orientation.w = 0.697;

    goal_dropoff_zone.target_pose.header.frame_id = "map";
    goal_dropoff_zone.target_pose.header.stamp = ros::Time::now();

    goal_dropoff_zone.target_pose.pose.position.x = -6.16;
    goal_dropoff_zone.target_pose.pose.position.y = -0.28;
    goal_dropoff_zone.target_pose.pose.orientation.w = 0.013;

    goal_home.target_pose.header.frame_id = "map";
    goal_home.target_pose.header.stamp = ros::Time::now();

    goal_home.target_pose.pose.position.x = -0.48;
    goal_home.target_pose.pose.position.y = -0.8;
    goal_home.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Moving towards pickup zone.");
    ac.sendGoal(goal_pickup_zone);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The pickup zone has been reached!");
        std_msgs::Bool pickup_notification;
        pickup_notification.data = true;
        pickup_publisher.publish(pickup_notification);
    }else{
        ROS_INFO("The robot failed to reach the pickup zone...");
    }
    ros::Duration(5.0).sleep();

    ROS_INFO("Moving towards dropoff zone.");
    ac.sendGoal(goal_dropoff_zone);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The dropoff zone has been reached!");
        std_msgs::Bool dropoff_notification;
        dropoff_notification.data = true;
        dropoff_publisher.publish(dropoff_notification);
        ros::Duration(5.0).sleep();
        ROS_INFO("Mission accomplished, going back home!");
    }else{
        ROS_INFO("The robot failed to reach the dropoff zone...");
    }

    ac.sendGoal(goal_home);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("The robot reached home!");
    }else{
        ROS_WARN("The robot failed to reach home!");
    }
    ros::Duration(5.0);

    return 0;
}
