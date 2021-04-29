#include <ros/ros.h>
#include <std_msgs/String.h>
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
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#define MAX_BOARD_PLACE 0.05
typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> PlaceClient;

//void table_callback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);

tf::TransformListener *listener_ptr;

struct Point_t {
    float x;
    float y;

    Point_t() {
        x = y = 0.0;
    }

};

void look_up();

bool set_collision_update(bool state);

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName);

bool PlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

bool FreeCanCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

double randBetweenTwoNum(int max, int min);

bool exec = false;
ros::ServiceClient *uc_client_ptr;
ros::Publisher pub_controller_command;
Point_t point;

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double x_current = 0;
double z_cuurent = 0;
double y_cuurent = 0;

float x_real =0;
float y_real =0;
float z_real =0;

float x_real_can =0;
float y_real_can =0;
float z_real_can =0;

float x_rel_person=0;
float y_rel_person=0;
float z_rel_person=0;




void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    z_cuurent = msg->pose.position.z;
    //ROS_INFO_STREAM(x_current);
    pose.push_back(msg);
}

void cb_person(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
      //tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w);
      //tf::Matrix3x3 m(q);
      //double roll, pitch, yaw;
      //m.getRPY(roll, pitch, yaw);
      //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
      x_real=req.markers[0].pose.pose.position.x;
      y_real=req.markers[0].pose.pose.position.y;
      z_real=req.markers[0].pose.pose.position.z;
      //ROS_INFO_STREAM(x_real);
    } // if
}

void cb_person_rel(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
      //tf::Quaternion q(req.markers[0].pose.pose.orientation.x, req.markers[0].pose.pose.orientation.y, req.markers[0].pose.pose.orientation.z, req.markers[0].pose.pose.orientation.w);
      //tf::Matrix3x3 m(q);
      //double roll, pitch, yaw;
      //m.getRPY(roll, pitch, yaw);
      //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
      // roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
      x_rel_person=req.markers[0].pose.pose.position.x;
      y_rel_person=req.markers[0].pose.pose.position.y;
      z_rel_person=req.markers[0].pose.pose.position.z;
      //ROS_INFO_STREAM(x_real);
    } // if
}

void cb_can(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
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
      //ROS_INFO_STREAM(x_real);
    } // if
}

//void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //ROS_INFO_STREAM("Received pose: " << msg);
    //x_current = -msg->pose.position.y;
    //z_cuurent = msg->pose.position.z;
    //y_cuurent = -msg->pose.position.x;

    //ROS_INFO_STREAM(x_current);
    //ROS_INFO_STREAM(y_cuurent);

    //pose.push_back(msg);
//}

int main(int argc, char **argv) {

    ros::init(argc, argv, "place_node_person");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle pn("~");



    

    // to get table pose relative to image coordinates - x_cuurent z_current
    ros::Subscriber subscribetf = n.subscribe("/find_object_node_person/object_pose_person", 1000, tf_callback); 

    pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);
    
    

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(n, "object_pose_person", 10);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/detected_objects_person", 1, cb_person);
    

    ros::NodeHandle nhc;
    ros::Subscriber sub_can = nhc.subscribe("/detected_objects", 1, cb_can);
    //tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    //tf_filter.registerCallback( boost::bind(table_callback, _1) );
    
    
             /*add moviet colision */

    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "person";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.3;
    primitive.dimensions[1] = 0.3;
    primitive.dimensions[2] = 0.01;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x_real; //-0.0
    box_pose.position.y = y_real;
    box_pose.position.z = z_real-0.2;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

 
    /*end addinf moviet colision */
    
    
    ros::ServiceServer pickAndPlace = n.advertiseService("place_go_person", &PlaceCallBack);

    ros::ServiceServer pickPlace = n.advertiseService("free_can", &FreeCanCallBack);

    ros::waitForShutdown();
    return 0;
}

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName) {
    moveit_msgs::PlaceGoal placeGoal;
    placeGoal.group_name = "arm";
    placeGoal.attached_object_name = objectName;
    placeGoal.place_eef = false;
    placeGoal.support_surface_name = "person";
    placeGoal.planner_id = "RRTConnectkConfigDefault";
    placeGoal.allowed_planning_time = 5.0;
    placeGoal.planning_options.replan = true;
    placeGoal.planning_options.replan_attempts = 5;
    placeGoal.planning_options.replan_delay = 2.0;
    placeGoal.planning_options.planning_scene_diff.is_diff = true;
    placeGoal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    std::vector<moveit_msgs::PlaceLocation> locations;
    moveit_msgs::PlaceLocation location;
    location.pre_place_approach.direction.header.frame_id = "/base_footprint";
    location.pre_place_approach.direction.vector.z = -0.1; //-0.1
    location.pre_place_approach.min_distance = 0.1;
    location.pre_place_approach.desired_distance = 0.2;

    location.post_place_retreat.direction.header.frame_id = "/gripper_link";
    //location.post_place_retreat.direction.vector.x = -0.15;
    location.post_place_retreat.direction.vector.x = -0.1;
    location.post_place_retreat.min_distance = 0.0;
    location.post_place_retreat.desired_distance = 0.2;

    location.place_pose.header.frame_id = placeGoal.support_surface_name;
    bool inTheBoard;
    
    //ROS_INFO_STREAM(x_current);
    do {
        inTheBoard = true;
        location.place_pose.pose.position.x = randBetweenTwoNum(1, -2); //10,-10
        //location.place_pose.pose.position.x = 0.05; //10,-10
        //location.place_pose.pose.position.x = x_real;
        
        //location.place_pose.pose.position.x =x_real/20.0; //20
        
        
       if(fabs(point.x + location.place_pose.pose.position.x) >= MAX_BOARD_PLACE) {
            inTheBoard = false;
        }
        //inTheBoard = true; //new
    } while (!inTheBoard);
    
    do {
        inTheBoard = true;
        location.place_pose.pose.position.y = randBetweenTwoNum(2, -2); // 10 -10
        //location.place_pose.pose.position.y = 0; // 10 -10
        //location.place_pose.pose.position.y =y_real;
        //ROS_INFO_STREAM(location);
        //location.place_pose.pose.position.y =y_real/20.0; //20
        if(fabs(point.y + location.place_pose.pose.position.y) >= MAX_BOARD_PLACE) {
            inTheBoard = false;
        }
        //inTheBoard = true; //new
    } while (!inTheBoard);

    location.place_pose.pose.position.z = 0.22; //0.12
    

    location.place_pose.pose.orientation.w = 1.0;
    //ROS_INFO_STREAM(location);
    
    

    locations.push_back(location);
    placeGoal.place_locations = locations;
    ROS_INFO_STREAM(placeGoal);


    


    return placeGoal;
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
        ROS_ERROR("Failed to call service /find_objects_node_person/update_colision");
        return false;
    }

}

bool PlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;

    pn.param<std::string>("object_name", object_name, "can");
    //pn.param<std::string>("object_name", object_name, "table");

    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }

   


    ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/plp/trigger",10);
    ros::Duration(1).sleep();






    
    
    std_msgs::String msg;
    msg.data = "start place";
    _chatter_pub.publish(msg);
    ros::Publisher chatter_flag = n.advertise<std_msgs::Int8>("/place_flag", 0);
    std_msgs::Int8 place_flag;
    place_flag.data=2;
    chatter_flag.publish(place_flag);

    geometry_msgs::Twist vel;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    ros::Publisher chatter16 = n.advertise<std_msgs::Float64>("/torso_effort_controller/command", 0.09);
    std_msgs::Float64 msg16;
    msg16.data = 0.38;
    chatter16.publish(msg16);
    ros::Duration(2.0).sleep();

    look_up();
    
    ROS_INFO("Looking at person...");
    ros::Duration(5.0).sleep();
    ROS_INFO("Giving object to person");
    ROS_INFO("wait...");
    
     while ((z_cuurent>0.75) or ((x_current<-0.08) or (x_current>0.08)))
   {
    
   if (x_current<-0.08)
    {
    while (x_current<-0.08)
    {
    vel.angular.z = 0.05;
    vel.linear.x = 0.0;
    pub.publish(vel);
    ros::Duration(1).sleep();
    
    }
    }
    else
    {
    if (x_current>0.08)
    {
    while (x_current>0.08)
    {
    vel.angular.z = -0.05;
    vel.linear.x = 0.0;
    pub.publish(vel);
    ros::Duration(1).sleep();

    }
    }
   
    if (z_cuurent>0.75)
    {
        
    while (z_cuurent>0.75)
    {
    vel.angular.z = 0.0;
    vel.linear.x = 0.05;
    pub.publish(vel);
    ros::Duration(2).sleep();
    }
    }
    
    
    }
   }
    
    
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "person";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.3;
    primitive.dimensions[1] = 0.3;
    primitive.dimensions[2] = 0.01;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x_real; //-0.0
    box_pose.position.y = y_real;
    box_pose.position.z = z_real-0.22;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    PlaceClient placeClient("place", true);
    placeClient.waitForServer();
    bool found = false;
    
    

    /* move to the wanted point -start */
    /* motion by place servise*/

    
    /*
    moveit_msgs::PlaceGoal placeGoal = buildPlaceGoal(object_name);

    
    actionlib::SimpleClientGoalState placeStatus = placeClient.sendGoalAndWait(placeGoal);

    
    
    if (placeStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
        found = true;
        res.success = (unsigned char) (found);
        res.message = placeStatus.getText();
        //point.x = placeGoal.place_locations[0].place_pose.pose.position.x+x_real;
        //point.y = placeGoal.place_locations[0].place_pose.pose.position.y+y_real;
        point.x += placeGoal.place_locations[0].place_pose.pose.position.x;
        point.y += placeGoal.place_locations[0].place_pose.pose.position.y;
        


    }

      */  
        //moveit::planning_interface::MoveGroupInterface move_group("arm");
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

         /* move to the wanted point -end */


        /* move the hand infront of person -start*/

        /* motion by joint-space goal */
        /*
        static const std::string PLANNING_GROUP = "arm";
        //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
        //current_state = move_group.getCurrentState();
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        joint_group_positions[0] = 0.0; //0.41  // radians // rotation1
        joint_group_positions[3] = 0.0; //1.51    //rotation2
        joint_group_positions[1] = -0.10; //0.97     //shoulder1
        joint_group_positions[2] = 0.59; //-1.11     //shoulder2
        joint_group_positions[4] = 0.35; //-1.52   //shoulder3
        joint_group_positions[5] = 0.0; //-2.56   //wrist
        
        move_group.setJointValueTarget(joint_group_positions);
        move_group.move();
        ros::Duration(1).sleep();*/
       
        /* move the hand infront of person - end*/


        const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup("arm");
        

        //joint_model_group =move_group.getCurrentState()->getJointModelGroup("arm");


        /* move the hand infront of person -start*/
        /* motion by xyz goal */
        
       static const std::string PLANNING_GROUP = "arm";
       //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
       moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
       //const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
       joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
       
       
        geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

        std::cout<< "Anfangsposition" << std::endl;
        std::cout<< target_pose3 << std::endl;

        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose3);

        
        //target_pose3.position.x += 0.3; // right
        target_pose3.position.y += 0.25; // forward
        target_pose3.position.z += 0.20; // up

        //target_pose3.position.y = (y_real- target_pose3.position.y ); // forward
        //target_pose3.position.z = (z_real- target_pose3.position.z ); // up

        waypoints.push_back(target_pose3);

        // Skalierungsfaktor der maximalen Geschwindigkeit jedes Gelenks
        move_group.setMaxVelocityScalingFactor(0.1);

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
        goal_plan.trajectory_ = trajectory;
        move_group.execute(goal_plan);
        ros::Duration(1).sleep();
        

       

        
        /* move the hand infront of person -end*/



        
       
    ROS_INFO("wait for next command");

    std_srvs::SetBool enableColl;
    enableColl.request.data = true;
    if(uc_client.call(enableColl)) {
        ROS_INFO("update_colision response: ON ");
    }

    msg.data = "finish place";
    _chatter_pub.publish(msg);
    ros::Duration(1).sleep();
    return true;


}

bool FreeCanCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    ros::NodeHandle pn("~");
    ros::NodeHandle n;
    std::string object_name;

    pn.param<std::string>("object_name", object_name, "can");
    //pn.param<std::string>("object_name", object_name, "table");

    ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
    std_srvs::SetBool disableColl;
    disableColl.request.data = false;

    if(uc_client.call(disableColl)) {
        ROS_INFO("update_colision response: OFF ");
    }

   


    ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/plp/trigger",10);
    ros::Duration(1).sleep();






    
    
    std_msgs::String msg;
   ROS_INFO("Take you'r can ! :) ");

   ros::Publisher grip16 = n.advertise<control_msgs::GripperCommandActionGoal>("/gripper_controller/gripper_cmd/goal", 0.1,0.1 );
    control_msgs::GripperCommandActionGoal grip_msg;
    grip_msg.goal.command.position = 0.1;
    
    grip16.publish(grip_msg);
    ros::Duration(2.0).sleep();



}


double randBetweenTwoNum(int max, int min) {
    int randomNum = rand()%(max-min + 1) + min;
    return randomNum / 100.0;
}


void look_up() {

    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("head_pan_joint");
    traj.joint_names.push_back("head_tilt_joint");
    traj.points.resize(1);
    traj.points[0].time_from_start = ros::Duration(1.0);
    std::vector<double> q_goal(2);
    q_goal[0]=0.0;
    q_goal[1]=0.15;
    traj.points[0].positions=q_goal;
    traj.points[0].velocities.push_back(0);
    traj.points[0].velocities.push_back(0);
    pub_controller_command.publish(traj);
}