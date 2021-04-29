#include "ros/ros.h"
#include "std_msgs/String.h"

std::string inputString;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "input_message_robot");

  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<std_msgs::String> ("/plp/trigger", 10);
  ros::Rate loop_rate(5);
  while(nh.ok())
  {
    std::cout << "Give input: t - to rotate: ";
    std::getline(std::cin, inputString);
    std_msgs::String msg;

    msg.data = inputString;
    if(inputString=="t")
    {
       p.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}
