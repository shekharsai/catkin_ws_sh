#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <robotican_demos_upgrade/switch_topic.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "demo_place_node");
    ros::NodeHandle n;

    ros::ServiceClient pick_client = n.serviceClient<std_srvs::Trigger>("place_go");
    //ros::ServiceClient sw_client = n.serviceClient<robotican_common::switch_topic>("switch_pcl_topic");

    ROS_INFO("Waiting for services...");
    pick_client.waitForExistence();
    //sw_client.waitForExistence();

    //robotican_common::switch_topic sw_srv;
    //sw_srv.request.num=1;
    //sw_client.call(sw_srv);

    //ros::Duration(5).sleep();
    //ROS_INFO("Ready!");

    //sw_srv.request.num=2;
    //sw_client.call(sw_srv);

    //ros::Duration(5).sleep();

    std_srvs::Trigger pick_srv;
    if (pick_client.call(pick_srv)) {
        ROS_INFO("pick response: %s", pick_srv.response.message.c_str());
        if (pick_srv.response.success)
        {
            //ros::Publisher _chatter_pub = n.advertise<std_msgs::String>("/robot_state",10);
           // ros::Duration(1).sleep();

            //std_msgs::String msg;
           // msg.data = "finish place";
            //_chatter_pub.publish(msg);
            ROS_INFO("Done!");
        }
    }
    else ROS_ERROR("Failed to call place_service");

    ros::shutdown();
    return 0;
}
