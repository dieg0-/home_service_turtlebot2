#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

class MarkerVisualizer{
    public:
        visualization_msgs::Marker object_marker;
  
        void init_communications(){
            marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
          
            while(marker_publisher_.getNumSubscribers() < 1){
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1.0);
            }
            ROS_INFO("A Subscriber to the marker has been created.");
            marker_publisher_.publish(object_marker);
          
            pickup_subscriber_ = node_handle_.subscribe("/pickup", 10, &MarkerVisualizer::pickup_clbk, this);
            dropoff_subscriber_ = node_handle_.subscribe("/dropoff", 10, &MarkerVisualizer::dropoff_clbk, this);
        }
  
        void init_marker(){
            uint32_t shape = visualization_msgs::Marker::CUBE;

            object_marker.header.frame_id = "/map";
            object_marker.header.stamp = ros::Time::now();

            object_marker.ns = "objects";
            object_marker.id = 0;

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

            object_marker.pose.position.x = 3.61;
            object_marker.pose.position.y = -4.68;
            object_marker.pose.position.z = 0;
            object_marker.pose.orientation.x = 0.0;
            object_marker.pose.orientation.y = 0.0;
            object_marker.pose.orientation.z = 0.0;
            object_marker.pose.orientation.w = 1.0;
        }
  
        void pickup_clbk(const std_msgs::Bool notification){
            ROS_INFO("Picking up the object!");
            object_marker.action = visualization_msgs::Marker::DELETE;
            marker_publisher_.publish(object_marker);
        }
  
        void dropoff_clbk(const std_msgs::Bool notification){
            ROS_INFO("Dropping off the object!");
            object_marker.action = visualization_msgs::Marker::ADD;
            object_marker.pose.position.x = -6.16;
            object_marker.pose.position.y = -0.28;
            marker_publisher_.publish(object_marker);
        }
  
    private:
        ros::NodeHandle node_handle_;
        ros::Publisher marker_publisher_; 
        ros::Subscriber pickup_subscriber_;
        ros::Subscriber dropoff_subscriber_;
};

int main( int argc, char** argv ){
    ros::init(argc, argv, "add_markers");

    MarkerVisualizer visualizer;
    visualizer.init_marker();
    visualizer.init_communications();
  
    ros::spin();    

    return 0;
}
