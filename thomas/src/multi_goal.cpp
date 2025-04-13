// Ce programme a pour but d'envoyer différents goal à MoveBase, comme le programme send_goal.cpp
// Les coordonnées des goals a envoyé sont lues depuis un fichier goals.yaml. Chaque coordonées ont un identifiant
// un autre fichier route.yaml contient une liste d'identifiant. Ce sont ceux des points que l'on souhaite envoyé. 
// Le programme stocke donc les coordonnées de goals.yaml, puis lie dans l'ordre les identifiant du fichier route.yaml et envoie les coordonnées correspondantes à MoveBase

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <map>

// Déclaration du client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Structure pour stocker un objectif
struct Goal {
    int id;
    double x, y, theta;
};

// Fonction pour charger les objectifs depuis goals.yaml
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

// Fonction pour charger l'ordre des objectifs depuis route.yaml
std::vector<int> loadRoute(const std::string& file_path) {
    std::vector<int> route;
    YAML::Node config = YAML::LoadFile(file_path);

    for (const auto& id : config["route"]) {
        route.push_back(id.as<int>());
    }
    return route;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_goal_publisher");
    ros::NodeHandle nh("~");
    
    
    // Les routes ont été rentré manuellement car le programme ne les trouvait pas en utilisant d'autre méthode
    // Ces routes sont à changé si le code est utilisé depuis un autre ordinateur
    std::string goals_path = "/home/thomas/catkin_ws/src/thomas/config/goals.yaml";
    std::string route_path = "/home/thomas/catkin_ws/src/thomas/config/route.yaml";
    
    // Les goals sont stocké dans une map
    std::map<int, Goal> goals = loadGoals(goals_path);
    // et la route dans un vecteur
    std::vector<int> route = loadRoute(route_path);

    // Le client se connecte à MoveBase
    MoveBaseClient ac("move_base", true);
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base server");

    // Pour chaque identifiant de route, on trouve les coordonnées dans la map pour les envoyer
    for (int id : route) {
        if (goals.find(id) != goals.end()) {
            Goal goal = goals[id];
            move_base_msgs::MoveBaseGoal moveGoal;

            moveGoal.target_pose.header.frame_id = "map";
            moveGoal.target_pose.header.stamp = ros::Time::now();
            moveGoal.target_pose.pose.position.x = goal.x;
            moveGoal.target_pose.pose.position.y = goal.y;
            moveGoal.target_pose.pose.orientation.w = cos(goal.theta / 2.0);
            moveGoal.target_pose.pose.orientation.z = sin(goal.theta / 2.0);

            ROS_INFO("Going to goal %d: x=%.2f, y=%.2f, theta=%.2f", goal.id, goal.x, goal.y, goal.theta);
            ac.sendGoal(moveGoal);
            // On attend que le robot soit arrivé au goal pour envoyé le suivant
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Goal %d reached!", goal.id);
            else
                ROS_WARN("Failed to reach goal %d.", goal.id);

            ros::Duration(2.0).sleep(); // Pause avant le prochain objectif
        } else {
            ROS_WARN("Goal ID %d not found in goals.yaml", id);
        }
    }

    ROS_INFO("All goals processed.");
    return 0;
}
