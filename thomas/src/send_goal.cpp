// Ce node a pour but de communiquer avec MoveBase. Le fait d'utiliser un client est plus 
// approprié qu'un simple publisher car il permet aussi de recevoir des informations sur 
// l'avancement des mouvements du robot.

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Création du client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nav_goal_publisher");
    // Le client est lié à "move_base" et attend d'être connecter pour continuer le programme
    MoveBaseClient ac("move_base", true);

    ROS_INFO("Waiting for the move_base action server...");
    ac.waitForServer(); // Attendre que move_base soit prêt
    ROS_INFO("Connected to move_base server");
    
    //Crée un message de type MoveBaseGoal
    move_base_msgs::MoveBaseGoal goal;

    // Définition du cadre de référence
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Définition des coordonnées X, Y et orientation du robot
    goal.target_pose.pose.position.x = 2.0;
    goal.target_pose.pose.position.y = 3.0;
    goal.target_pose.pose.orientation.w = 1.0;  // Orientation en radian

    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);

    // Attente de la fin de l'exécution
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached successfully!");
    else
        ROS_WARN("Failed to reach goal.");

    return 0;
}
