#include <ros/ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


bool set_collision_update(bool state);
bool DrivePoseCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool GivePoseCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


ros::ServiceClient *uc_client_ptr;
ros::Publisher pub_controller_command;

int main(int argc, char **argv){
    
    ros::init(argc, argv, "poses");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    std::string object_name,table_name;
    std::string EndPositionName ;
    std::string GivePositionName ;
    
    pn.param<std::string>("end_position_name", EndPositionName, "pre_grasp2");
    pn.param<std::string>("give_position_name", GivePositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    pn.param<std::string>("table_name", table_name, "table");
    
    ros::ServiceServer DrivePose = n.advertiseService("drive_pose", &DrivePoseCallback);
    ros::ServiceServer GivePose = n.advertiseService("give_pose", &GivePoseCallback);
    ros::waitForShutdown();
    
}

bool set_collision_update(bool state){
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

bool DrivePoseCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;
    
    pn.param<std::string>("object_name", object_name, "can");
    
    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = true;
    
    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }
    
    ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/plp/trigger",10);
    ros::Duration(1).sleep();
    
    std_msgs::String msg;
    msg.data = "start to move";
    _chatter_pub.publish(msg);
    
    std::string EndPositionName;
    
    pn.param<std::string>("end_position_name", EndPositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    moveit::planning_interface::MoveGroup group("arm");
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(50);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    
    group.setStartStateToCurrentState();
    group.setNamedTarget(EndPositionName);
    moveit::planning_interface::MoveGroup::Plan EndPosPlan;
    
    if(group.plan(EndPosPlan)) { //Check if plan is valid
        group.execute(EndPosPlan);
        ROS_INFO("move to pose");
    }
    
    else {
        ROS_ERROR("Error");
    }
    
    return true;
    
}


bool GivePoseCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;
    
    pn.param<std::string>("object_name", object_name, "can");
    
    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = true;
    
    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }
    
    ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/plp/trigger",10);
    ros::Duration(1).sleep();
    
    std_msgs::String msg;
    msg.data = "start to move";
    _chatter_pub.publish(msg);
    
    std::string GivePositionName;
    
    pn.param<std::string>("give_position_name", GivePositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    moveit::planning_interface::MoveGroup group_1("arm");
    group_1.setPlanningTime(10.0);
    group_1.setNumPlanningAttempts(50);
    group_1.setPlannerId("RRTConnectkConfigDefault");
    group_1.setPoseReferenceFrame("base_footprint");
    
    group_1.setStartStateToCurrentState();
    group_1.setNamedTarget(GivePositionName);
    moveit::planning_interface::MoveGroup::Plan GivePosPlan;
    
    if(group_1.plan(GivePosPlan)) { //Check if plan is valid
        group_1.execute(GivePosPlan);
        ROS_INFO("move to pose");
    }
    
    else {
        ROS_ERROR("Error");
    }
    
    return true;
    
}