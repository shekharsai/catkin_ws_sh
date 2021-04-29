#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <robotican_demos_upgrade/switch_topic.h>

using namespace cv;
bool have_object=false;
bool have_object_one=false;
int object_id = 1;

std::string depth_topic1 = "/kinect2/qhd/points";
std::string depth_topic2 = "/softkinetic_camera/depth/points";
std::string depth_topic = "";

image_transport::Publisher result_image_pub;
image_transport::Publisher object_image_pub;
image_transport::Publisher bw_image_pub;

ros::Publisher pose_pub;

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,Point3d *obj,std::string frame);

//red
int minH=3,maxH=160;
int minS=70,maxS=255;

int minV=10,maxV=255;
int minA=200,maxA=50000;
int gaussian_ksize=0;
int gaussian_sigma=0;
int morph_size=0;

int inv_H=1;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    //pcl - point clound library with lot of algorithms

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
    Mat result=cv_ptr->image;

    Point3d obj;
    //find object
    have_object= find_object(result,cloudp,&obj,input->header.frame_id);

    waitKey(1);

    if (have_object && !have_object_one) {
        have_object_one = true;
    //publish geometry msg containing object's location
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id=input->header.frame_id;
        target_pose.header.stamp=ros::Time::now();
        target_pose.pose.position.x =obj.x;
        target_pose.pose.position.y = obj.y;
        target_pose.pose.position.z = obj.z;
        target_pose.pose.orientation.w=1;

        pose_pub.publish(target_pose);
 //ROS_INFO_STREAM("find_object x: " <<  target_pose.pose.position.x);
    }
//    else
//    {
//        geometry_msgs::PoseStamped target_pose1;
//        target_pose1.pose.position.x = 0;
//        pose_pub.publish(target_pose1);
//    }
}

bool find_object(Mat input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp,Point3d *pr,std::string frame) {

    Mat hsv,filtered,bw,mask;

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=  frame;
    //use hsv colores - no light sensitivity
    cvtColor(input,hsv,CV_BGR2HSV);


    if (inv_H) {
        //edges of spectrom - red colore
        Mat lower_hue_range;
        Mat upper_hue_range;
        inRange(hsv, cv::Scalar(0, minS, minV), cv::Scalar(minH, maxS, maxV), lower_hue_range);
        inRange(hsv, cv::Scalar(maxH, minS, minV), cv::Scalar(179, maxS, maxV), upper_hue_range);
        // Combine the above two images

        addWeighted(lower_hue_range, 1.0, upper_hue_range, 1.0, 0.0, mask);
    }
    else{
        //if not red use (middle of spectrom):
        inRange(hsv,Scalar(minH,minS,minV),Scalar(maxH,maxS,maxV),mask);
    }
    hsv.copyTo(filtered,mask);
    cvtColor(filtered,filtered,CV_HSV2BGR);

    out_msg.image    = filtered;
    out_msg.encoding = "bgr8";
    object_image_pub.publish(out_msg.toImageMsg());
    //convert to bw image, gaussian - blur, morphologic actions
    mask.copyTo(bw);
    if (gaussian_ksize>0) {
        if (gaussian_ksize % 2 == 0) gaussian_ksize++;
        GaussianBlur( bw, bw, Size(gaussian_ksize,gaussian_ksize), gaussian_sigma , 0);
    }


    if (morph_size>0) {
        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
        morphologyEx( bw, bw, MORPH_CLOSE, element, Point(-1,-1), 1 );
    }

    out_msg.image    = bw;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    bw_image_pub.publish(out_msg.toImageMsg());

    //get image contours
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;

    findContours(bw, contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    //get largest contour
    double largest_area=0;
    int largest_contour_index=0;
    for( int i = 0; i< contours.size(); i++ )
    {
        double area0 = abs(contourArea(contours[i]));
        if(area0>largest_area){
            largest_area=area0;
            largest_contour_index=i;
        }
    }
    bool ok=false;
    if ((largest_area>minA)&&(largest_area<maxA)) {

        //draw contours and details about object
        drawContours(input, contours, (int)largest_contour_index,  Scalar(255,0,0), 3, 8, hierarchy, 0);
        Moments mu=moments( contours[largest_contour_index], true );
        Point2f mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        circle( input, mc, 4, Scalar(0,0,255), -1, 8, 0 );
        int pcl_index = ((int)(mc.y)*input.cols) + (int)(mc.x);
        circle( input, mc, 8, Scalar(0,255,0), -1, 8, 0 );

        pr->x=cloudp->points[pcl_index].x;
        pr->y=cloudp->points[pcl_index].y;
        pr->z=cloudp->points[pcl_index].z;

        char str[100];
        //ROS_INFO("Find Object");
        if (std::isnan (pr->x) || std::isnan (pr->y) || std::isnan (pr->z) ) {
            sprintf(str,"NaN");
            ok=false;
        }
        else {
            sprintf(str,"[%.3f,%.3f,%.3f] A=%lf",pr->x,pr->y,pr->z,largest_area);
            ok=true;

        }
        putText( input, str, mc, CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(255,255,255), 1, 8);

    }

    out_msg.image    = input;
    out_msg.encoding = "bgr8";
    result_image_pub.publish(out_msg.toImageMsg());
    return ok;
}

bool switch_pcl_topic(robotican_demos_upgrade::switch_topic::Request &req, robotican_demos_upgrade::switch_topic::Response &res) {

    if (req.num==1) depth_topic=depth_topic1;
    else if (req.num==2) depth_topic=depth_topic2;
    res.success=true;
return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_find_object");

  ros::NodeHandle n;

  ros::NodeHandle pn("~");

  pn.param<int>("object_id", object_id, 1);
  pn.param<std::string>("depth_topic1", depth_topic1, "/kinect2/qhd/points");
  pn.param<std::string>("depth_topic2", depth_topic2, "/softkinetic_camera/depth/points");
  depth_topic=depth_topic1;

  image_transport::ImageTransport it_(pn);

  result_image_pub = it_.advertise("result", 1);
  object_image_pub = it_.advertise("hsv_filterd", 1);
  bw_image_pub = it_.advertise("bw", 1);

  ros::Subscriber pcl_sub = n.subscribe(depth_topic, 1, cloud_cb);

  pose_pub=pn.advertise<geometry_msgs::PoseStamped>("object_pose",10);



  ros::ServiceServer switch_sub = n.advertiseService("switch_pcl_topic", &switch_pcl_topic);

  //system("/home/lab/script/script3.sh");
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
