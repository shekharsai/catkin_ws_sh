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
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
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

bool set_collision_update(bool state);

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName);

bool PlaceCallBack(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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




void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    z_cuurent = msg->pose.position.z;
    //ROS_INFO_STREAM(x_current);
    pose.push_back(msg);
}

void cb_table(ar_track_alvar_msgs::AlvarMarkers req) {
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

    /*
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "/kinect2_link";
    pose_msg.pose.position.x=x_current;
    pose_msg.pose.position.y=y_cuurent;
    pose_msg.pose.position.z=z_cuurent;
    pose_msg.pose.orientation.w=1;

    tf::TransformListener listener;
    listener.lookupTransform("/base_footprint", pose_msg) -> pose_msg;

    x_current = pose_msg.pose.position.x;
    z_cuurent = pose_msg.pose.position.z;
    y_cuurent = pose_msg.pose.position.y;
    */
    //ROS_INFO_STREAM(x_current);
    //ROS_INFO_STREAM(y_cuurent);

    //pose.push_back(msg);
//}
/*
void table_callback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr) {
    //x_real=point_ptr.markers[0].pose.pose.position.x;
    //y_real=point_ptr.markers[0].pose.pose.position.y;
    //z_real=point_ptr.markers[0].pose.pose.position.z;

    //try
    //{
        geometry_msgs::PoseStamped base_object_pose;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
        x_real=point_ptr->pose.position.x;
        y_real=point_ptr->pose.position.y;
        z_real=point_ptr->pose.position.z;
    //}
     // catch (tf::TransformException &ex)
    //{
      //  printf ("Failure %s\n", ex.what()); //Print exception which was caught
    //}
}

*/
int main(int argc, char **argv) {

    ros::init(argc, argv, "place_node_table");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    srand((unsigned int) time(NULL));
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    /*ros::NodeHandle pn("~");
    std::string object_name,table_name;
    std::string startPositionName ;

    pn.param<std::string>("start_position_name", startPositionName, "pre_grasp2");
    pn.param<std::string>("object_name", object_name, "can");
    pn.param<std::string>("table_name", table_name, "table");
    */


    //ros::Subscriber subscribetf = n.subscribe("/find_object_node_table/object_pose_table", 1000, tf_callback);

    // to get table pose relative to image coordinates - x_cuurent z_current
    ros::Subscriber subscribetf = n.subscribe("/find_object_node_table/object_pose_table", 1000, tf_callback); 
    

    message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
    point_sub_.subscribe(n, "object_pose_table", 10);

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/detected_objects_table", 1, cb_table);

    ros::NodeHandle nhc;
    ros::Subscriber sub_can = nhc.subscribe("/detected_objects", 1, cb_can);
    //tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
    //tf_filter.registerCallback( boost::bind(table_callback, _1) );
    
    
             /*add moviet colision */

    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "table1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.3;
    primitive.dimensions[1] = 0.3;
    primitive.dimensions[2] = 0.01;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x_real-0.3; //-0.2
    box_pose.position.y = y_real;
    box_pose.position.z = z_real;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    //moveit_msgs::CollisionObject collision_object;
    //moveit::planning_interface::MoveGroupInterface move_group("arm");
    //collision_object.header.frame_id = move_group.getPlanningFrame();
    /* can addition */
    /*
    collision_object.id = "can";
    shape_msgs::SolidPrimitive object_primitive;
    primitive.type = primitive.CYLINDER;
    object_primitive.dimensions.resize(2);
	object_primitive.dimensions[0] = 0.145;
	object_primitive.dimensions[1] = 0.03;
    geometry_msgs::Pose can_pose;
    can_pose.orientation.w = 1.0;
    can_pose.position.x = x_real_can; //-0.0
    can_pose.position.y = y_real_can;
    can_pose.position.z = z_real_can;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(can_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    */
    /*end addinf moviet colision */
    
    
    ros::ServiceServer pickAndPlace = n.advertiseService("place_go_table", &PlaceCallBack);
    /*ROS_INFO("Hello");
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
    if(group.plan(startPosPlan)) { //Check if plan is valid
        group.execute(startPosPlan);
        pub_controller_command = n.advertise<trajectory_msgs::JointTrajectory>("/pan_tilt_trajectory_controller/command", 2);
        ROS_INFO("Waiting for the moveit action server to come up");
        ros::ServiceClient uc_client = n.serviceClient<std_srvs::SetBool>("update_collision_objects");
        ROS_INFO("Waiting for update_collision service...");
        uc_client.waitForExistence();
        uc_client_ptr = &uc_client;
        //set_collision_update(true);
        ros::Duration(10.0).sleep();
        ROS_INFO("Ready!");
    }
    else {
        ROS_ERROR("Error");
    }
    */
    ros::waitForShutdown();
    return 0;
}

moveit_msgs::PlaceGoal buildPlaceGoal(const std::string &objectName) {
    moveit_msgs::PlaceGoal placeGoal;
    placeGoal.group_name = "arm";
    placeGoal.attached_object_name = objectName;
    placeGoal.place_eef = false;
    placeGoal.support_surface_name = "table";
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
    location.pre_place_approach.direction.vector.z = -0.05; //-0.1
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
        location.place_pose.pose.position.x = randBetweenTwoNum(-4, -10); //10,-10
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
        location.place_pose.pose.position.y = randBetweenTwoNum(1, -1); // 2 -2
        //location.place_pose.pose.position.y = 0; // 10 -10
        //location.place_pose.pose.position.y =y_real;
        //ROS_INFO_STREAM(location);
        //location.place_pose.pose.position.y =y_real/20.0; //20
        if(fabs(point.y + location.place_pose.pose.position.y) >= MAX_BOARD_PLACE) {
            inTheBoard = false;
        }
        //inTheBoard = true; //new
    } while (!inTheBoard);

    location.place_pose.pose.position.z = 0.12; //0.10
    //location.place_pose.pose.position.z=z_real+0.15;

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
        ROS_ERROR("Failed to call service /find_objects_node_table/update_colision");
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
    

    ros::Publisher chatter16_new = n.advertise<std_msgs::Float64>("/torso_effort_controller/command", 1000,true);
    std_msgs::Float64 msg16;
    msg16.data = z_real-0.32;
    for (int ttt=0;ttt<2;ttt++)
    {
    chatter16_new.publish(msg16);
    ros::Duration(1).sleep();
    }
    ros::Duration(2).sleep();
    
     while ((z_cuurent>1.12) or ((x_current<-0.08) or (x_current>0.08)))
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
   
    if (z_cuurent>1.12)
    {
        
    while (z_cuurent>1.12)
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
    collision_object.id = "table1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.3;
    primitive.dimensions[1] = 0.3;
    primitive.dimensions[2] = 0.01;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = x_real-0.35; //-0.3
    box_pose.position.y = y_real;
    box_pose.position.z = z_real+0.22;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

        PlaceClient placeClient("place", true);
        placeClient.waitForServer();
        bool found = false;
        /*do {
            moveit_msgs::PlaceGoal placeGoal = buildPlaceGoal(object_name);
            actionlib::SimpleClientGoalState placeStatus = placeClient.sendGoalAndWait(placeGoal);
            if (placeStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
                found = true;
                res.success = (unsigned char) (found);
                res.message = placeStatus.getText();
                point.x = placeGoal.place_locations[0].place_pose.pose.position.x;
                point.y = placeGoal.place_locations[0].place_pose.pose.position.y;
                //point.x += placeGoal.place_locations[0].place_pose.pose.position.x;
                //point.y += placeGoal.place_locations[0].place_pose.pose.position.y;
                


            }
        } while(!found);*/

        moveit_msgs::PlaceGoal placeGoal = buildPlaceGoal(object_name);
            actionlib::SimpleClientGoalState placeStatus = placeClient.sendGoalAndWait(placeGoal);
            ros::Duration(2).sleep();
            if (placeStatus == actionlib::SimpleClientGoalState::SUCCEEDED) {
                found = true;
                res.success = (unsigned char) (found);
                ros::Duration(2).sleep();
                res.message = placeStatus.getText();
                //point.x = placeGoal.place_locations[0].place_pose.pose.position.x+x_real;
                //point.y = placeGoal.place_locations[0].place_pose.pose.position.y+y_real;
                point.x += placeGoal.place_locations[0].place_pose.pose.position.x;
                point.y += placeGoal.place_locations[0].place_pose.pose.position.y;
                


            }


        //moveit::planning_interface::MoveGroupInterface move_group("arm");
        //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        const robot_state::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup("arm");

         /*move back before cobra center */
        vel.angular.z = 0.0;
        vel.linear.x = -0.08;
        pub.publish(vel);
        ros::Duration(3).sleep();
        
        //come back to "cobra_center" after place
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
double randBetweenTwoNum(int max, int min) {
    int randomNum = rand()%(max-min + 1) + min;
    return randomNum / 100.0;
}
