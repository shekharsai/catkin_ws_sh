#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

bool update_cb(std_srvs::SetBool::Request  &req,std_srvs::SetBool::Response &res);
void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers);
std::string get_obj_id(int id);
shape_msgs::SolidPrimitive get_obj_shape(int id);

bool update=false;

moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_ptr;

//listening to alvar msgs, that are sent througth detected_objects node

int main(int argc, char **argv) {
    ros::init(argc, argv, "objects_handler");
    ros::NodeHandle n;

    ros::Subscriber color_sub = n.subscribe("detected_objects", 10, handle_objects);
    ros::ServiceServer service = n.advertiseService("update_collision_objects", update_cb);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface_ptr=&planning_scene_interface;

while (ros::ok()){
    ros::spinOnce();
}
    return 0;
}

std::string get_obj_id(int id) { //TODO: FROM DB
    if (id==1) return "can";
    else return "table";

}

shape_msgs::SolidPrimitive get_obj_shape(int id) { //TODO: FROM DB

shape_msgs::SolidPrimitive object_primitive;
if (id==1) {
//object_primitive.type = object_primitive.SPHERE;
//object_primitive.dimensions.resize(2);
//object_primitive.dimensions[0] = 0.1;
//object_primitive.dimensions[1] = 0.01;
object_primitive.type = object_primitive.CYLINDER;
object_primitive.dimensions.resize(2);
object_primitive.dimensions[0] = 0.145;
object_primitive.dimensions[1] = 0.03;
}
else {
   //object_primitive.type = object_primitive.BOX;
   //object_primitive.dimensions.resize(2);
   //object_primitive.dimensions[0] = 0.2;
   //object_primitive.dimensions[1] = 0.2;
   //object_primitive.dimensions[2] = 0.01;

     object_primitive.type = object_primitive.BOX;
     object_primitive.dimensions.resize(3);
     object_primitive.dimensions[0] = 0.3;
     object_primitive.dimensions[1] = 0.3;
     object_primitive.dimensions[2] = 0.01;
}
    return object_primitive;
}

void handle_objects(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& markers)
{
	//build collisions for moveit
    std::vector<moveit_msgs::CollisionObject> col_objects;

    for (int i=0;i<markers->markers.size();i++) {

        ar_track_alvar_msgs::AlvarMarker m=markers->markers[i];

        moveit_msgs::CollisionObject obj;
        obj.header.frame_id = m.header.frame_id;
        obj.id = get_obj_id(m.id);
        obj.primitives.push_back(get_obj_shape(m.id));

        obj.primitive_poses.push_back(m.pose.pose);
        obj.operation=obj.ADD;

        col_objects.push_back(obj);

        if (update) planning_scene_interface_ptr->addCollisionObjects(col_objects);

    }
}

bool update_cb(std_srvs::SetBool::Request  &req,
               std_srvs::SetBool::Response &res) {

    update=req.data;
    res.success=true;
    if (update)  res.message="update collision is ON";
    else res.message="update collision is OFF";
    return true;
}
