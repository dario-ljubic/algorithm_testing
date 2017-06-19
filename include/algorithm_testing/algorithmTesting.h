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
#include "schunk_lwa4p_trajectory/WaypointArray.h"
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
    
//     Eigen::MatrixXd goal_q_old;
    Eigen::MatrixXd lwa4p_temp_q;
    
    // Algorithm parameters
    double d = 500; // distance from the camera to the observer
    float zeroThreshold = 0.001; // margin
    //bool movingToPoint = false; // indicates if a robot is moving towards the point
    //bool firstTime = true; // indicates if the robot is moving first time
    
    // starting position
    double x0 = 500;
    double y0 = 0;
    double z0 = 1000;
    
    // Gaze data
    double horGaze;
    double verGaze;
    
};