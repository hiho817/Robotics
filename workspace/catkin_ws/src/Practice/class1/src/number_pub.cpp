#include "ros/ros.h"
#include "std_msgs/Int32.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "number_pub3");
  ros::NodeHandle n;
  ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int32>("/number_3", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Int32 msg;
    cin >> msg.data;
    chatter_pub1.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
