#include <iostream>
#include <queue>
#include <string>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <chrono>

// ROS
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>

// user defined
#include "gazetool/GazeHyps.h"
#include <schunk_lwa4p_kinematics/lwa4p_kinematics.h>

class algorithmTesting
{
public:
    algorithmTesting();
    ~algorithmTesting();
    void run();  
    
private:
    void jointStatesCallback(const sensor_msgs::JointState &msg);
    void gazeCallback(const gazetool::GazeHyps& msg);
    void initializeKinematics();
    void initializePosition();
    void move();
    
    // Node handle, publishers and subscriber
    ros::NodeHandle n;
    ros::Publisher pub_arm_1;
    ros::Publisher pub_arm_2;
    ros::Publisher pub_arm_3;
    ros::Publisher pub_arm_4;
    ros::Publisher pub_arm_5;
    ros::Publisher pub_arm_6;
    ros::Subscriber jointStatesSub;
    ros::Subscriber gazeSub;
    
    // lwa4p_kinematics
    lwa4p_kinematics kinematic;
    int robot_id = 0; // to choose blue robot (important when loading kinematic parameters)
    
    Eigen::MatrixXd lwa4p_temp_q;
    std_msgs::Float64 goal_q1, goal_q2, goal_q3, goal_q4, goal_q5, goal_q6;
    
    // Algorithm parameters
    double d = 500; // distance from the camera to the observer
    double x0 = 500; // starting position
    double y0 = 0;
    double z0 = 1000;
    
    // Gaze data
    double horGaze;
    double verGaze;
    
};