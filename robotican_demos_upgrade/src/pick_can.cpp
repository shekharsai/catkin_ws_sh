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


#define MAX_BOARD_PLACE 0.05
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickClient;

struct Point_t {
    float x;
    float y;

    Point_t() {
        x = y = 0.0;
    }

};

void look_down();
void trosso_set();

bool set_collision_update(bool state);

moveit_msgs::PickupGoal BuildPickGoal(const std::string &objectName);

moveit_msgs::PickupGoal BuildCheckkGoal(const std::string &objectName);

bool pickAndPlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

bool exec = false;
ros::ServiceClient *uc_client_ptr;
ros::Publisher pub_controller_command;
ros::Publisher chatter16;
ros::Publisher chatter16_new;



Point_t point;

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double x_current = 0.0;
double z_cuurent = 0.0;

float x_real_can =0.0;
float y_real_can =0.0;
float z_real_can =0.0;


void cb_can(ar_track_alvar_msgs::AlvarMarkers req) 
{
    if (!req.markers.empty()) 
    {
        //tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w);
        //tf::Matrix3x3 m(q);
        //double roll, pitch, yaw;
        //m.getRPY(roll, pitch, yaw);
        //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
        // roll  --> rotate around vertical axis
        // pitch --> rotate around horizontal axis
        // yaw   --> rotate around depth axis
        x_real_can=req.markers[0].pose.pose.position.x;
        y_real_can=req.markers[0].pose.pose.position.y;
        z_real_can=req.markers[0].pose.pose.position.z;

        // ROS_INFO("There is a can in the frame!");
    } 
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    //ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    z_cuurent = msg->pose.position.z;
    //ROS_INFO_STREAM(x_current);
    pose.push_back(msg);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "pick_and_plce_node");
    ros::AsyncSpinner spinner(2);
    ros::NodeHandle nh;
    
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle nhc;
    //ros::NodeHandle nh;
    ros::NodeHandle pn("~");
    std::string object_name,table_name;
    std::string startPositionName ;
    ros::Subscriber subscribetf = n.subscribe("/find_object_node/object_pose", 1000, tf_callback);
    ros::Subscriber sub_can = nhc.subscribe("/detected_objects", 1, cb_can);
    //ros::Subscriber subscribetf = n.subscribe("/myPoint", 1000, tf_callback);
    
    
    pn.param<std::string>("start_position_name", startPositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    pn.param<std::string>("table_name", table_name, "table");

    ros::ServiceServer pickAndPlace = n.advertiseService("pick_go", &pickAndPlaceCallBack);
    ROS_INFO("Hello");
    moveit::planning_interface::MoveGroup group("arm");
    //Config move group
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(10);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");

    group.setStartStateToCurrentState();
    group.setNamedTarget(startPositionName);
    moveit::planning_interface::MoveGroup::Plan startPosPlan;

    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    
    

    if(group.plan(startPosPlan)) 
    {   //Check if plan is valid
        group.execute(startPosPlan);
        pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);
        ros::Publisher chatter16 = n.advertise<std_msgs::Float64>("/torso_effort_controller/command", 0.09);
        //chatter16 = n.advertise<std_msgs::Float64>("/torso_effort_controller/command", 0.09);
        
        
        ROS_INFO("Waiting for the moveit action server to come up");
        ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
        ROS_INFO("Waiting for update_collision service...");
        uc_client.waitForExistence();
        uc_client_ptr = &uc_client;
        set_collision_update(true);
        ros::Duration(8.0).sleep();
        look_down();
        ROS_INFO("Looking down...");
        ros::Duration(5.0).sleep();
       // trosso_set();
        std_msgs::Float64 msg16;
        msg16.data = 0.22;
        chatter16.publish(msg16);
        
        ros::Publisher chatter_flag = n.advertise<std_msgs::Int8>("/place_flag", 0);
        
        std_msgs::Int8 place_flag;
        place_flag.data=0;
        for (int i=0; i<2; i++)
        {
        chatter_flag.publish(place_flag);
        }
        ros::Duration(5.0).sleep();
        ROS_INFO("set torso pose...");
        ROS_INFO("Ready!");
        //cout<<x_current;
        //ROS_INFO_STREAM(x_current);
        
        //ros::Rate rate(1);  
        
    }
    else {
        ROS_ERROR("Error");
    }
    ros::waitForShutdown();
    return 0;
}

moveit_msgs::PickupGoal BuildPickGoal(const std::string &objectName) 
{   
    moveit_msgs::PickupGoal goal;
   
    goal.target_name = objectName;
    goal.group_name = "arm";
    goal.end_effector = "eef";
    goal.allowed_planning_time = 5.0;
    goal.planning_options.replan_delay = 2.0;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.replan=true;
    goal.planning_options.replan_attempts=5;
    goal.planner_id = "RRTConnectkConfigDefault";

    goal.minimize_object_distance = true;
    moveit_msgs::Grasp g;
    g.max_contact_force = 1.00; //0.01
    g.grasp_pose.header.frame_id = goal.target_name;
    g.grasp_pose.pose.position.x = -0.02;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = -0.02;
    g.grasp_pose.pose.orientation.x = 0.00;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "gripper_link"; //"//base_footprint"; //gripper_link
    g.pre_grasp_approach.direction.vector.x = 0.92; //1.0
    g.pre_grasp_approach.min_distance = 0.1;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.post_grasp_retreat.direction.header.frame_id = "gripper_link"; //base_footprint"; //gripper_link
    g.post_grasp_retreat.direction.vector.z = 0.52; //0.49
    g.post_grasp_retreat.direction.vector.y = 0.2;
    //g.post_grasp_retreat.direction.vector.x = -0.1;

    
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.4;
    goal.possible_grasps.push_back(g);   

    /*
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup("arm");

    
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.38;  // radians
    joint_group_positions[1] = 1.06;
    joint_group_positions[2] = 1.24;
    joint_group_positions[3] = -0.32;
    joint_group_positions[4] = -1.30;
    joint_group_positions[5] = -1.26;
    
    move_group.setJointValueTarget(joint_group_positions);
    */
    
    return goal;
}




void look_down() 
{
    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=0.0;
    q_goal[1]=0.6;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    pub_controller_command.publish(traj);
}

void trosso_set() 
{

    std_msgs::Float64 msg16;
    
    msg16.data = 0.4;
    chatter16.publish(msg16);
    
}

bool set_collision_update(bool state)
{
    std_srvs::SetBool srv;
    srv.request.data=state;
    if (uc_client_ptr->call(srv))
    {
        ROS_INFO("update_colision response: %s", srv.response.message.c_str());
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service /find_objects_node/update_colision");
        return false;
    }

}

bool pickAndPlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) 
{
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;

    pn.param<std::string>("object_name", object_name, "can");

    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }

    ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/plp/trigger",10);
    ros::Duration(1).sleep();

    std_msgs::String msg;
    msg.data = "start pick";
    _chatter_pub.publish(msg);

    ros::Publisher chatter16_new = n.advertise<std_msgs::Float64>("/torso_effort_controller/command", 1000,true);
    std_msgs::Float64 msg16;
    msg16.data = z_real_can-0.5;
    for (int ttt=0;ttt<2;ttt++)
    {
    chatter16_new.publish(msg16);
    ros::Duration(1).sleep();
    }
    ros::Duration(2).sleep();
    geometry_msgs::Twist vel;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
       
   while ((z_cuurent>0.9) or ((x_current<-0.05) or (x_current>0.05)))
   {
    
   if (x_current<-0.05)
    {
    while (x_current<-0.05)
    {
    vel.angular.z = 0.05;
    vel.linear.x = 0.0;
    pub.publish(vel);
    ros::Duration(1).sleep();
    
    }
    }
    else
    {
    if (x_current>0.05)
    {
    while (x_current>0.05)
    {
    vel.angular.z = -0.05;
    vel.linear.x = 0.0;
    pub.publish(vel);
    ros::Duration(1).sleep();

    }
    }
   
    if (z_cuurent>0.9)
    {
        
    while (z_cuurent>0.9)
    {
    vel.angular.z = 0.0;
    vel.linear.x = 0.05;
    pub.publish(vel);
    ros::Duration(2).sleep();
    }
    }
    
    
    }
   }   
    
    
    //pub.publish(vel); 
    PickClient pickClient("pickup", true);
    pickClient.waitForServer();
    bool found = false;
    /*
    do {
        moveit_msgs::PickupGoal pickGoal = BuildPickGoal(object_name);
        actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);
        
        if(pickStatus != actionlib::SimpleClientGoalState::SUCCEEDED) {
            //res.success = (unsigned char) false;
            found = true;
            res.success = (unsigned char) (found);
            res.message = pickStatus.getText();
            point.x = point.y = 0;
        }
    //ros::Duration(5.0).sleep();
    }
    
    while(!found);
    
    */

   moveit_msgs::PickupGoal pickGoal = BuildPickGoal(object_name);
      
   
   actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);
   ros::Duration(2.0).sleep();
    if(pickStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //res.success = (unsigned char) false;
            found = true;
            res.success = (unsigned char) (found);
            ros::Duration(2.0).sleep();
            res.message = pickStatus.getText();
            point.x = point.y = 0;

                if (!res.success)
                {

                actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);  
                }
    }
    else
    {
        std::cout << "Failed to pick the object!" << std::endl;
        ros::Duration(2.0).sleep();

        std::cout << "Will try to find out the reason!" << std::endl;

        ros::Subscriber sub_can = n.subscribe("/detected_objects", 1, cb_can);
        
        // don't remove the below if-else
        if(x_real_can != 0.0 && y_real_can != 0.0 && z_real_can != 0.0)
        {
            std::cout << "Failed due to an external factor!" << std::endl;
            ros::Duration(2.0).sleep();
        }
        else
        {
            std::cout << "The object is not in the frame!" << std::endl;
            ros::Duration(2.0).sleep();
        }        
    }

    
    /*move to position for object recognation */
    /*
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup("arm");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.48; //0.41  // radians // rotation1
    joint_group_positions[3] = 1.43; //1.51    //rotation2
    joint_group_positions[1] = 1.12; //0.97     //shoulder1
    joint_group_positions[2] = -1.38; //-1.11     //shoulder2
    joint_group_positions[4] = -1.56; //-1.52   //shoulder3
    joint_group_positions[5] = -2.68; //-2.56   //wrist
    
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();
    
    */

   
    /*move back before cobra center */
    vel.angular.z = 0.0;
    vel.linear.x = -0.08;
    pub.publish(vel);
    ros::Duration(7).sleep();


    //come back to "cobra_center" after pick
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup("arm");

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.0; //0.41  // radians // rotation1
    joint_group_positions[3] = 0.0; //1.51    //rotation2
    joint_group_positions[1] = 0.0; //0.97     //shoulder1
    joint_group_positions[2] = 2.1838; //-1.11     //shoulder2
    joint_group_positions[4] = -1.3802; //-1.52   //shoulder3
    joint_group_positions[5] = 0.0; //-2.56   //wrist
    
    move_group.setJointValueTarget(joint_group_positions);
    move_group.move();

    msg16.data = 0.22;
    for (int ttt=0;ttt<2;ttt++)
    {
    chatter16_new.publish(msg16);
    ros::Duration(1).sleep();
    }
    ros::Duration(2).sleep();


    /*PickClient pickClient("pickup", true);
    pickClient.waitForServer();

    moveit_msgs::PickupGoal pickGoal = BuildPickGoal(object_name);
    actionlib::SimpleClientGoalState pickStatus = pickClient.sendGoalAndWait(pickGoal);
    if(pickStatus != actionlib::SimpleClientGoalState::SUCCEEDED) {
        //res.success = (unsigned char) false;
        res.success = (unsigned char) true;
        res.message = pickStatus.getText();
        point.x = point.y = 0;
    }
    else {
        PlaceClient placeClient("place", true);
        placeClient.waitForServer();
        bool found = false;
        do {
            moveit_msgs::PlaceGoal placeGoal = buildPlaceGoal(object_name);
            actionlib::SimpleClientGoalState placeStatus = placeClient.sendGoalAndWait(placeGoal);
            if (placeStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
                found = true;
                res.success = (unsigned char) (found);
                res.message = placeStatus.getText();
                point.x += placeGoal.place_locations[0].place_pose.pose.position.x;
                point.y += placeGoal.place_locations[0].place_pose.pose.position.y;
            }
        } while(!found);
    }

    std_srvs::SetBool enableColl;
    enableColl.request.data = true;
    if(uc_client.call(enableColl)) {
        ROS_INFO("update_colision response: ON ");
    }*/

    return true;


}
