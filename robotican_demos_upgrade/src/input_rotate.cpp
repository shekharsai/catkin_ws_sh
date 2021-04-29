#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <robotican_demos_upgrade/switch_topic.h>
#include <time.h>
#include <string>

bool flagtodrive = false;
bool objectnotfound = false;

ros::Publisher pub;
ros::Publisher pub_cmd;
std_msgs::String msg_str;
int count = 0;
ros::ServiceClient resetworld_client;
float temp = 0.0;

float angularMotionOnly() {

    count ++;
    //ROS_INFO("count: %i", count);
    if(count > 920)
    //if(count > 100)
    {
        objectnotfound = true;
        msg_str.data = "not find object";
        pub.publish(msg_str);
        ros::Duration(2).sleep();
        //system("/home/lab/script/script.sh");
    }
    // return 0.0;
    //return (rand()%100 + 10)*(((float)22/7)/180);
    temp += 0.000001;
    return (5)*(((float)22/7)/180);// + (temp);
}

void Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Message: [%s]", msg->data.c_str());
  std::string inputString = msg->data.c_str();
  if(inputString=="t")
  {
      count = 0;
      objectnotfound = false;
      ROS_INFO("t");
      ros::NodeHandle nh;

      //Ceates the publisher, and tells it to publish
      //to the /cmd_vel topic, with a queue size of 100
      pub_cmd=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
      ros::Rate rate(10);
      while(nh.ok() && flagtodrive == false && objectnotfound == false)
      {
      //ROS_INFO("Turning");
      geometry_msgs::Twist msg;
      //x value between -2 and 2
      msg.linear.x=0.0;
      //y value between -3 and 3
      msg.angular.z = angularMotionOnly();//0.2;
      //ROS_INFO("angular: %f", msg.angular.z);
      //Publish the message
      pub_cmd.publish(msg);
      ros::spinOnce();
      rate.sleep();
      }
  }
}

void drive()
{
    ros::Duration(2).sleep();
    ROS_INFO("Waiting For Service To go There");

    ros::NodeHandle np;
    ros::ServiceClient drive_client = np.serviceClient<std_srvs::Trigger>("my_drive2object_go");

    ROS_INFO("Waiting for service...");
    drive_client.waitForExistence();
    ros::Duration(7).sleep();
    ROS_INFO("Ready!");
    msg_str.data = "start drive";
    pub.publish(msg_str);
    std_srvs::Trigger drive_srv;
    if (drive_client.call(drive_srv))
    {

        ROS_INFO("my_drive2object_go response: %s", drive_srv.response.message.c_str());
        if (drive_srv.response.success) {
//            msg_str.data = "finish drive";
//            pub.publish(msg_str);
//            ROS_INFO("Done!");

//            ros::Duration(2).sleep();

            //system("/home/lab/script/script2.sh");

            //exit(0);

            //system("pkill -f /opt/ros");

            /*system("pkill robot_state_publisher");*/

            /*pid_t pid;
            pid = fork();
            printf("%d member of %d\n", getpid(), getpgrp());

            //system("killall -9 " + getpid());
            system("kill -SIGINT" + getpid());

            //system("killall -9 roscore");
            //system("killall -9 rosmaster");


            //system("killall roscore");

            ros::Duration(20).sleep();

            system("roslaunch robotican_armadillo armadillo1.launch kinect2:=true softkinetic:=true gmapping:=true hector_slam:=true move_base:=true lidar:=true world_name:=\"rospack find generate_srv /worlds/objects_on_table1.world\" gazebo:=true");
            //system("pkill roslaunch");
            ros::Duration(2).sleep();*/

            //ros::Duration(5).sleep();

            //add pick also

            ros::ServiceClient pick_client = np.serviceClient<std_srvs::Trigger>("pick_go");
            ros::ServiceClient sw_client = np.serviceClient<robotican_demos_upgrade::switch_topic>("switch_pcl_topic");

            ROS_INFO("Waiting for services...");
            pick_client.waitForExistence();
            sw_client.waitForExistence();

            robotican_demos_upgrade::switch_topic sw_srv;
            sw_srv.request.num=1;
            sw_client.call(sw_srv);

            ros::Duration(5).sleep();
            ROS_INFO("Ready!");

            sw_srv.request.num=2;
            sw_client.call(sw_srv);

            ros::Duration(5).sleep();

            std_srvs::Trigger pick_srv;
            //|| pick_srv.response.message=="No motion plan found. No execution attempted."
            if (pick_client.call(pick_srv)) {
                ROS_INFO("pick response: %s", pick_srv.response.message.c_str());
                if (pick_srv.response.success)
                {
                if(pick_srv.response.message=="Preempted" || pick_srv.response.message=="Solution found but controller failed during execution")
                    msg_str.data = "failed pick";
                else
                    msg_str.data = "finish pick";
                pub.publish(msg_str);
                ros::Duration(2).sleep();
                ROS_INFO("Done!");

                ros::Duration(5).sleep();

                //system("/home/lab/script/script.sh");

                }
           }
           else ROS_ERROR("Failed to call pick service");
        }
    }
    else ROS_ERROR("Failed to call drive2object service");
}

void handle_poses(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  //ROS_INFO_STREAM("Working");
  //if(firstflag)
  //{
  //ROS_INFO_STREAM("Received pose x: " <<  msg->pose.position.x);
  //ROS_INFO_STREAM("Received pose y: " <<  msg->pose.position.y);
  //ROS_INFO_STREAM("Received pose z: " <<  msg->pose.position.z);
  //firstflag = false;

   if(flagtodrive == false && msg->pose.position.x != 0)
  {
    msg_str.data = "find object";
    pub.publish(msg_str);
    ros::Duration(2).sleep();
    flagtodrive = true;
    // open it if you want to drive to the can
    drive();
  }
  //for search always can
 /*if(msg->pose.position.x == 0 && flagtodrive == true)
  {
     flagtodrive = false;
     msg_str.data = "t";
     pub.publish(msg_str);
     ros::Duration(2).sleep();
  }*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_queues");
  //system("/home/lab/script/script3.sh");
  ros::NodeHandle n;
  // nothing special for internal ROS queue
  ros::Subscriber sub = n.subscribe("/plp/trigger", 10, Callback);
  pub = n.advertise<std_msgs::String>("/plp/trigger",10);
  pub_cmd=n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  // define user callback queue
  ros::CallbackQueue string_queue;
  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
  ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
      //"find_object/object_pose", // topic name
      "my_find_object/object_pose", // topic name
      10, // queue length
      handle_poses, // callback
      ros::VoidPtr(), // tracked object, we don't need one thus NULL
      &string_queue // pointer to callback queue object
    );
  // subscribe
  ros::Subscriber sub2 = n.subscribe(ops);

  // spawn async spinner with 1 thread, running on our custom queue
  ros::AsyncSpinner async_spinner(1, &string_queue);
  // start the spinner
  async_spinner.start();

  //ros::Duration(5).sleep();

  //msg_str.data = "t";
  //pub.publish(msg_str);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
