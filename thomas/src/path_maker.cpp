// Ce programme est un mélange de multi_goal.cpp et de path_listener.cpp
// Il envoie différents goals à MoveBase en prennant en référence les coordonnées entrées dans goals.yaml
// mais la liste des goals a envoyé ne vient plus d'un fichier . yaml mais est lue dans le topic tsp_path envcoyé par path_finding.cpp

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <string>
#include <map>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Structure pour charger les goals
struct Goal {
    int id;
    double x, y, theta;
};

// Fonction qui charge les goals depuis goals.yaml
std::map<int, Goal> loadGoals(const std::string& file_path) {
    std::map<int, Goal> goals;
    YAML::Node config = YAML::LoadFile(file_path);

    for (const auto& goal : config["goals"]) {
        Goal g;
        g.id = goal["id"].as<int>();
        g.x = goal["x"].as<double>();
        g.y = goal["y"].as<double>();
        g.theta = goal["theta"].as<double>();
        goals[g.id] = g;
    }
    return goals;
}
// Cette fois, le programme doit avoir la même structure qu'un subscriber
class PathMaker {
public:
    PathMaker(const ros::NodeHandle& nh, const std::string& yaml_path)
        : nh_(nh), ac_("move_base", true) {
        // Charge les goals
        goals_ = loadGoals(yaml_path);
        // Souscrit au topic
        sub_ = nh_.subscribe("/tsp_path", 1, &PathMaker::tspCallback, this);

        ROS_INFO("Waiting for move_base action server...");
        ac_.waitForServer();
        ROS_INFO("Connected to move_base server");
    }
    // Cette fonction ne sera lancé que lorsque que le topic sera lu, son contenu est dans "msg"
    void tspCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
        ROS_INFO("Received TSP path with %lu points", msg->data.size());
	
	// Les identifiants reçu sont cherchée parmi tous les goals, puis envoyés à MoveBase
        for (int id : msg->data) {
            if (goals_.find(id) == goals_.end()) {
                ROS_WARN("Goal ID %d not found in YAML file", id);
                continue;
            }

            Goal goal = goals_[id];
            move_base_msgs::MoveBaseGoal moveGoal;

            moveGoal.target_pose.header.frame_id = "map";
            moveGoal.target_pose.header.stamp = ros::Time::now();
            moveGoal.target_pose.pose.position.x = goal.x;
            moveGoal.target_pose.pose.position.y = goal.y;
            moveGoal.target_pose.pose.orientation.w = cos(goal.theta / 2.0);
            moveGoal.target_pose.pose.orientation.z = sin(goal.theta / 2.0);

            ROS_INFO("Sending goal %d: x=%.2f, y=%.2f, theta=%.2f", id, goal.x, goal.y, goal.theta);
            ac_.sendGoal(moveGoal);
            ac_.waitForResult();

            if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Goal %d reached!", id);
            else
                ROS_WARN("Failed to reach goal %d.", id);

            ros::Duration(1.0).sleep();
        }

        ROS_INFO("Finished processing all TSP goals.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    MoveBaseClient ac_;
    std::map<int, Goal> goals_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_maker");
    ros::NodeHandle nh("~");

    std::string yaml_path;
    nh.param<std::string>("goals_yaml", yaml_path, "");

    if (yaml_path.empty()) {
        ROS_ERROR("goals_yaml param not set. Exiting.");
        return 1;
    }

    PathMaker pm(nh, yaml_path);
    ros::spin();
    return 0;
}

