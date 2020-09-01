#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tuple>
#include <vector>
#include <visualization_msgs/Marker.h>

#include "pick_objects/MissionRequest.h"

class MarkerVisualizer{
    public:
        //visualization_msgs::Marker object_marker;
        int current_marker = 0;
  
        std::vector<visualization_msgs::Marker> object_markers;
        std::vector<std::tuple<double, double, double>> pickup_positions = {std::make_tuple(3.61, -4.68, 0.697),
                                                                            std::make_tuple(-3.443, 2.721, 0.686),
                                                                            std::make_tuple(-2.282, -5.223, -0.043)};

        std::vector<std::tuple<double, double, double>> dropoff_positions = {std::make_tuple(-7.161, -0.337, -0.003),
                                                                             std::make_tuple(-6.608, -0.378, -0.003),
                                                                             std::make_tuple(-6.16, -0.28, 0.013)};
  
        bool assign_mission_clbk(pick_objects::MissionRequest::Request& request, 
                                    pick_objects::MissionRequest::Response& response){
            ROS_INFO("[MarkerVisualizer] A new mission has been requested.");
            if(current_marker >= pickup_positions.size()){
                ROS_INFO("[MarkerVisualizer] There are no more missions to assign.");
                response.granted.data = false;
            }else{
                ROS_INFO("[MarkerVisualizer] A new mission has been assigned.");
                response.granted.data = true;
              
                response.pickup_pose.position.x = std::get<0>(pickup_positions[current_marker]);
                response.pickup_pose.position.y = std::get<1>(pickup_positions[current_marker]);
                response.pickup_pose.orientation.w = std::get<2>(pickup_positions[current_marker]);
              
                response.dropoff_pose.position.x = std::get<0>(dropoff_positions[current_marker]);
                response.dropoff_pose.position.y = std::get<1>(dropoff_positions[current_marker]);
                response.dropoff_pose.orientation.w = std::get<2>(dropoff_positions[current_marker]);         
            }
            return true;
        }
  
        void init_communications(){
            marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            while(marker_publisher_.getNumSubscribers() < 1){
                ROS_WARN_ONCE("[MarkerVisualizer] Please create a subscriber to the marker.");
                sleep(1.0);
            }
            ROS_INFO("[MarkerVisualizer] A Subscriber to the marker has been created.");
            for(visualization_msgs::Marker object_marker : object_markers)
                marker_publisher_.publish(object_marker);
          
            pickup_subscriber_ = node_handle_.subscribe("/pickup", 10, &MarkerVisualizer::pickup_clbk, this);
            dropoff_subscriber_ = node_handle_.subscribe("/dropoff", 10, &MarkerVisualizer::dropoff_clbk, this);
          
            mission_server_ = node_handle_.advertiseService("/request_mission", &MarkerVisualizer::assign_mission_clbk, this);
        }
  
        void init_marker(){
            int id_count = 0;

            for(std::tuple<double, double, double> position : pickup_positions){
                visualization_msgs::Marker object_marker;    
                uint32_t shape = visualization_msgs::Marker::CUBE;

                object_marker.header.frame_id = "/map";
                object_marker.header.stamp = ros::Time::now();

                object_marker.ns = "objects";
                object_marker.id = id_count;
                id_count++;

                object_marker.type = shape;

                object_marker.scale.x = 0.3;
                object_marker.scale.y = 0.3;
                object_marker.scale.z = 0.3;

                object_marker.color.r = 0.8f;
                object_marker.color.g = 0.0f;
                object_marker.color.b = 0.8f;
                object_marker.color.a = 1.0;

                object_marker.lifetime = ros::Duration();

                object_marker.action = visualization_msgs::Marker::ADD;

                object_marker.pose.position.x = std::get<0>(position);
                object_marker.pose.position.y = std::get<1>(position);
                object_marker.pose.position.z = 0;
                object_marker.pose.orientation.x = 0.0;
                object_marker.pose.orientation.y = 0.0;
                object_marker.pose.orientation.z = 0.0;
                object_marker.pose.orientation.w = 1.0;

                object_markers.push_back(object_marker);
            }

        }
  
        void pickup_clbk(const std_msgs::Bool notification){
            ROS_INFO("[MarkerVisualizer] Picking up the object!");
            object_markers[current_marker].action = visualization_msgs::Marker::DELETE;
            marker_publisher_.publish(object_markers[current_marker]);
        }
  
        void dropoff_clbk(const std_msgs::Bool notification){
            ROS_INFO("[MarkerVisualizer] Dropping off the object!");
            object_markers[current_marker].action = visualization_msgs::Marker::ADD;
            object_markers[current_marker].pose.position.x = std::get<0>(dropoff_positions[current_marker]);
            object_markers[current_marker].pose.position.y = std::get<1>(dropoff_positions[current_marker]);
            marker_publisher_.publish(object_markers[current_marker]);
            current_marker++;
        }
  
    private:
        ros::NodeHandle node_handle_;
        ros::Publisher marker_publisher_; 
        ros::Subscriber pickup_subscriber_;
        ros::Subscriber dropoff_subscriber_;
        ros::ServiceServer mission_server_;
};

int main( int argc, char** argv ){
    ros::init(argc, argv, "add_markers");
    ROS_INFO("[MarkerVisualizer] Running!");
    MarkerVisualizer visualizer;
    visualizer.init_marker();
    visualizer.init_communications();
  
    ros::spin();    

    return 0;
}
