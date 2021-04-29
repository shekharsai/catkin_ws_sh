


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>


using namespace std;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);


std::string depth_topic1,depth_topic2,depth_topic;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //convert ros point cloud msg to pcl point cloud 
    pcl::fromROSMsg (*input, cloud); 
    //create projection image from p.c.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {
        ROS_WARN("empty cloud");
        return;
    }
	//creating new ros sensor msg - picture is relative to depth camera tf
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;
		
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    //cv::Mat result=cv_ptr->image;
    cv::namedWindow( "A");
    //imshow("A",result); 
    //waitKey(0);
   
}



int main(int argc, char **argv) {

    ros::init(argc, argv, "find_objects_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
	
    pn.param<std::string>("depth_topic1", depth_topic1, "/kinect2/qhd/points");
    pn.param<std::string>("depth_topic2", depth_topic2, "/kinect2/qhd/points");
    depth_topic=depth_topic1;


    ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
    ROS_INFO_STREAM(depth_topic);

    ros::Rate r(10);
    ROS_INFO("Ready to find objects!");
    while (ros::ok()) {
    if (pcl_sub.getTopic()!=depth_topic) {
        pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);
        ROS_INFO("switching pcl topic");
    }
     ros::spinOnce();
     r.sleep();
    }

    return 0;
}

