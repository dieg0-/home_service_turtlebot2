#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "pick_objects/MissionRequest.h"
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ObjectPicker{
    public:
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> * move_base_client = NULL;

        move_base_msgs::MoveBaseGoal current_pickup_position;
        move_base_msgs::MoveBaseGoal current_dropoff_position;
        move_base_msgs::MoveBaseGoal home;

        ros::Publisher pickup_publisher_;
        ros::Publisher dropoff_publisher_;
        ros::ServiceClient request_mission_client_;

    private:
        ros::NodeHandle node_handle_;        
        
    public:
        void init_communications(){
            home.target_pose.header.frame_id = "map";
            home.target_pose.header.stamp = ros::Time::now();

            home.target_pose.pose.position.x = -0.738;
            home.target_pose.pose.position.y = -1.038;
            home.target_pose.pose.orientation.w = 1.0;

            move_base_client = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(node_handle_, "move_base", true);
            pickup_publisher_ = node_handle_.advertise<std_msgs::Bool>("/pickup", 10);
            dropoff_publisher_ = node_handle_.advertise<std_msgs::Bool>("/dropoff", 10);
            request_mission_client_ = node_handle_.serviceClient<pick_objects::MissionRequest>("/request_mission");
        }

        bool request_mission(){
            bool response = true;
            pick_objects::MissionRequest service_request;
            service_request.request.request.data = true;
            if(request_mission_client_.call(service_request)){
                ROS_INFO("[ObjectPicker] Call to service succeded!");
                if(service_request.response.granted.data == true){
                    current_pickup_position.target_pose.header.frame_id = "map";
                    current_pickup_position.target_pose.header.stamp = ros::Time::now();

                    current_pickup_position.target_pose.pose.position.x = service_request.response.pickup_pose.position.x;
                    current_pickup_position.target_pose.pose.position.y = service_request.response.pickup_pose.position.y;
                    current_pickup_position.target_pose.pose.orientation.w = service_request.response.pickup_pose.orientation.w;

		    current_dropoff_position.target_pose.header.frame_id = "map";
                    current_dropoff_position.target_pose.header.stamp = ros::Time::now();

                    current_dropoff_position.target_pose.pose.position.x = service_request.response.dropoff_pose.position.x;
                    current_dropoff_position.target_pose.pose.position.y = service_request.response.dropoff_pose.position.y;
                    current_dropoff_position.target_pose.pose.orientation.w = service_request.response.dropoff_pose.orientation.w;
                }else{
                    response = false;
                }
            }else{
                ROS_INFO("[ObjectPicker] Call to service failed!");
            }
            return response;
        }

};



int main(int argc, char** argv){
    ros::init(argc, argv, "pick_objects");
    ROS_INFO("[ObjectPicker] Running!");
    	
    ObjectPicker picker;
    picker.init_communications();
    
    while(!picker.move_base_client->waitForServer(ros::Duration(5.0))){
        ROS_INFO("[ObjectPicker] Waiting for the move_base action server...");
    }
    
    while(!picker.request_mission_client_.waitForExistence()){
        ROS_INFO("[ObjectPicker] Waiting for the mission server...");
    }

    ros::Rate rate(10);
    while(ros::ok()){
        bool service_response = picker.request_mission();
        if(service_response){
            ROS_INFO("[ObjectPicker] A new mission has been received.");
            picker.current_pickup_position.target_pose.header.stamp = ros::Time::now();
            picker.move_base_client->sendGoal(picker.current_pickup_position);
            picker.move_base_client->waitForResult();

            if(picker.move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("[ObjectPicker] The pickup zone has been reached, picking up object...");
                std_msgs::Bool pickup_notification;
                pickup_notification.data = true;
                picker.pickup_publisher_.publish(pickup_notification);
            }else{
                ROS_INFO("[ObjectPicker] The robot failed to reach the pickup zone...");
            }
            ros::Duration(5.0).sleep();

            ROS_INFO("[ObjectPicker] Moving towards dropoff zone.");
            picker.current_dropoff_position.target_pose.header.stamp = ros::Time::now();
            picker.move_base_client->sendGoal(picker.current_dropoff_position);
            picker.move_base_client->waitForResult();

            if(picker.move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("[ObjectPicker] The dropoff zone has been reached, dropping off the object...");
                std_msgs::Bool dropoff_notification;
                dropoff_notification.data = true;
                picker.dropoff_publisher_.publish(dropoff_notification);
                ros::Duration(5.0).sleep();
            }else{
                ROS_INFO("[ObjectPicker] The robot failed to reach the dropoff zone...");
            }
        }else{
            ROS_INFO("[ObjectPicker] There are no more missions, going back home: (%f, %f, %f)", 
                                     picker.home.target_pose.pose.position.x,
                                     picker.home.target_pose.pose.position.y,
                                     picker.home.target_pose.pose.orientation.w);
            picker.home.target_pose.header.stamp = ros::Time::now();
            picker.move_base_client->sendGoal(picker.home);
            picker.move_base_client->waitForResult();

            if(picker.move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("[ObjectPicker] The robot reached home!");
            }else{
                ROS_WARN("[ObjectPicker] The robot failed to reach home!");
            }
            ros::Duration(5.0);
            ros::shutdown();
        }
        rate.sleep();
    }

    return 0;
}
