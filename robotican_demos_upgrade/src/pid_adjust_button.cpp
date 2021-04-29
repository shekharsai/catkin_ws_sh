#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematics_metrics/kinematics_metrics.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/SetBool.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <std_msgs/Int8.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


#include <vector>
#include <sstream>
#include <geometry_msgs/Twist.h>
using namespace std;

#include<nav_msgs/Odometry.h>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

// set the global variable for x and z kinect-coordinate of the center of the the button. 
double x_current = 0.0;
double z_cuurent = 0.0;

/**
set global varible odometry x linear velocity
set global varible odometry z angular velocity
**/
float odom_lin_x =0.0;
float odom_ang_z =0.0;
// ros::NodeHandle n("pid_adjust_button");

void adjust_robot_location()
{
    // set of  pid controller
    
    ros::NodeHandle nts;

    geometry_msgs::Twist vel;
    ros::Publisher pub = nts.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    
    float eror_linear_x =0.0;
    float eror_angular_z =0.0;
    
    float i_z=0.0;
    float i_x=0.0;
    
    float eror_z_temp = 0.0;
    float eror_x_temp = 0.0;

    float d_x = 0.0 ;
    float d_z = 0.0 ;

    float ref_vel=0.02;

    float kp =1.25; //1.5
    float kp_angular=0.9;
    float ki = 0.3;
    float kd = 0.5;
    
    std::cout << "Before while: the current value of x_cur : " << x_current << " and z_cur : " << z_cuurent << std::endl;   

    while ((z_cuurent > 0.82) || (x_current < -0.2) || (x_current > 0.2)) // 0.93 0.23
    {
        std::cout << "In while: the current value of x_cur : " << x_current << " and z_cur : " << z_cuurent << std::endl;   
        if (x_current<-0.2)
        {
            ROS_INFO("1");
            while (x_current<-0.2)
            {
                ROS_INFO("correct to right");
                eror_angular_z =  ref_vel - odom_ang_z ;
                
                d_z = eror_angular_z - eror_z_temp;

                eror_z_temp = eror_angular_z ;


                vel.angular.z = eror_angular_z*kp_angular +i_z*ki ;
                i_z=i_z  +eror_angular_z; 
                //vel.angular.z = 0.02;

                vel.linear.x = 0.0;

                    if (vel.angular.z >0.3)
                {
                    vel.angular.z=0.3;
                }
                pub.publish(vel);
                ros::Duration(1).sleep();            
            }

        }
        else
        {
            if (x_current>0.2)
            {
                ROS_INFO("correct to right");
                while (x_current>0.2)
                {
                    ROS_INFO("correct to left");
                    eror_angular_z =  -ref_vel - odom_ang_z ;

                    d_z = eror_angular_z - eror_z_temp;

                    eror_z_temp = eror_angular_z ;
                    
                    vel.angular.z = (eror_angular_z*kp_angular +i_z*ki) ;
                    i_z=i_z  +eror_angular_z;

                    //vel.angular.z = -0.02;
                    
                    vel.linear.x = 0.0;
                    if (vel.angular.z <-0.3)
                    {
                        vel.angular.z=-0.3;
                    }
                    pub.publish(vel);
                    ros::Duration(1).sleep();
                }
                }
        }
        if (z_cuurent>0.82)
        {                
            while (z_cuurent>0.82)
            {
                ROS_INFO("one step forward");   
                vel.angular.z = 0.0;
                
                //vel.linear.x = 0.02;
                eror_linear_x =  ref_vel - odom_lin_x ;

                d_x = eror_linear_x - eror_x_temp;

                eror_x_temp = eror_linear_x ;
                
                vel.linear.x = eror_linear_x*kp +i_x*ki ;
                i_x=i_x  +eror_linear_x; 
                if (vel.linear.x >0.3)
                {
                    vel.linear.x=0.3;
                }

                pub.publish(vel);
                ros::Duration(1).sleep();
            }
        }    
        
    }

    ROS_INFO("Armadillo is at the right place!");
    std::cout << "return success : True!" << std::endl;    
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    // ROS_INFO("In tf_callback, received pose!");
    // std::cout << msg->pose.position.x << std::endl;
    x_current = msg->pose.position.x;
    z_cuurent = msg->pose.position.z;
    // std::cout << x_current << " " << z_cuurent << std::endl;
    pose.push_back(msg);

    adjust_robot_location();
}

void odom_callback(nav_msgs::Odometry odom)
{
    // ROS_INFO("In odom_callback, received pose!");
    odom_lin_x=odom.twist.twist.linear.x;
    odom_ang_z=odom.twist.twist.angular.z;
}



int main(int argc, char **argv) 
{
    ros::init(argc, argv, "pid_adjust_button");
    // ros::AsyncSpinner spinner(2);
    
    ros::NodeHandle n;
    ros::NodeHandle nt;
    // spinner.start();

    // subscribe to kinect coordinates - just change it to right node to subscribe the pose
    ros::Subscriber subscribetf = n.subscribe("/find_objects_ele_button/object_pose_button", 1000, tf_callback);

    // subscribe to robot odometry
    ros::Subscriber sub_odom= nt.subscribe("/mobile_base_controller/odom", 1000, odom_callback);

    ROS_INFO("In pid controller button!");
    std::cout << "Main: the current value of x_cur : " << x_current << " and z_cur : " << z_cuurent << std::endl;    

    ros::spin();
    return 0;
}



