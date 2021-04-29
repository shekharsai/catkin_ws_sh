#include "ros/ros.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>


tf::TransformListener *listener_ptr;

int object_id = 1;

ros::Publisher object_pub;

ros::Publisher object_pub1;

void obj_msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
{
    //ROS_INFO_STREAM("Received pose x: " <<  point_ptr->pose.position.x);
    //get object location msg
    try
    {
        geometry_msgs::PoseStamped base_object_pose;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);
        base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

        //simulate alvar msgs, to get tf
        ar_track_alvar_msgs::AlvarMarkers msg;
        msg.header.stamp=base_object_pose.header.stamp;
        msg.header.frame_id="base_footprint";

        ar_track_alvar_msgs::AlvarMarker m;
        m.id=object_id;
        m.header.stamp=base_object_pose.header.stamp;
        m.header.frame_id="base_footprint";
        m.pose=base_object_pose;
        msg.markers.push_back(m);
        m.pose.pose.position.z-=0.1;
        m.id=2;
        msg.markers.push_back(m);

        object_pub.publish(msg);

        geometry_msgs::PoseStamped base_object_pose1;
        listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose1);
        base_object_pose1.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0);


        //ROS_INFO("Did the transformation");
        //printf("X: %lf Y: %lf Z: %lf\n",base_object_pose1.pose.position.x,base_object_pose1.pose.position.y,base_object_pose1.pose.position.z);

        object_pub1.publish(base_object_pose1);


    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detected_objects");

  ros::NodeHandle n;

  object_pub=n.advertise<ar_track_alvar_msgs::AlvarMarkers>("detected_objects", 2, true);
  object_pub1=n.advertise<geometry_msgs::PoseStamped>("detected_object1", 2, true);

  //convert depth cam tf to base foot print tf (moveit work with base foot print tf)
  tf::TransformListener listener;
  listener_ptr=&listener;

  message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
  point_sub_.subscribe(n, "my_find_object/object_pose", 10);

  //message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_(n, "object_pose", 10);

  tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "base_footprint", 10);
  tf_filter.registerCallback( boost::bind(obj_msgCallback, _1) );


  ros::Rate r(10);
  while (ros::ok()) {
   ros::spinOnce();
   r.sleep();
  }

  return 0;
}

