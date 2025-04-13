// Ce programme a été créé pour tester le programme path-finding.cpp
// Il lie le contenu du topic "tsp_path", c'est un simple subscriber

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

void callback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    ROS_INFO("Réception du chemin TSP :");
    for (size_t i = 0; i < msg->data.size(); ++i) {
        ROS_INFO(" -> %d", msg->data[i]);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("tsp_path", 10, callback);
    ros::spin();
    return 0;
}
