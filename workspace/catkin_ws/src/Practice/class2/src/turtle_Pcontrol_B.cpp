// include ros library
#include "ros/ros.h"

// include msg library
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

// include math 
#include <math.h>

using namespace std;

float theta_error;

// turtle pose
turtlesim::Pose turtle;

// goal points
geometry_msgs::Point turtle_goal;

// turtle twist
geometry_msgs::Twist turtle_twist;

// turtle publisher
ros::Publisher turtle_pub;


bool reset;

struct XY{
    float x;
	float y;
};

struct XY pos_err_I;



// declare call back function
void turtle_cb(const turtlesim::Pose::ConstPtr& msg)
{
	turtle = *msg;
}

void worldtobodyQuat(float x, float y, float theta)
{
	/////////////// please uncomment the following code and finish it //////////////
	double w = cos(theta/2);
	double z = sin(theta/2);

	Eigen::Quaterniond q(w, 0, 0, z);
	Eigen::Quaterniond q_normalized(q.w()/q.norm(), q.x()/q.norm(), q.y()/q.norm(), q.z()/q.norm());
	Eigen::Quaterniond v(0, x, y, 0);
	Eigen::Quaterniond v_new = q_normalized.inverse()* v * q_normalized;
	pos_err_I.x = v_new.x();
	pos_err_I.y = v_new.y();
	std::cout << "x quat: " << pos_err_I.x << std::endl;
	std::cout << "y quat: " << pos_err_I.y << std::endl;
}

void body2WorldQuat(float &x, float &y, float theta)
{
	double w = cos(-theta/2);
	double z = sin(-theta/2);

	Eigen::Quaterniond q(w, 0, 0, z);
	Eigen::Quaterniond q_normalized(q.w()/q.norm(), q.x()/q.norm(), q.y()/q.norm(), q.z()/q.norm());
	Eigen::Quaterniond v(0, x, y, 0);
	Eigen::Quaterniond v_new = q_normalized.inverse()* v * q_normalized;
	turtle_goal.x = v_new.x()+turtle.x;
	turtle_goal.y = v_new.y()+turtle.y;
}

// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &turtle_pose, geometry_msgs::Twist &turtle_vel_msg) {

	// error in body frame
	pos_err_I.x = goal.x - turtle_pose.x;
	pos_err_I.y = goal.y - turtle_pose.y;

	// Find the goal_point position in Body(turtlesim) frame
	worldtobodyQuat(pos_err_I.x, pos_err_I.y, turtle_pose.theta);

	// Find the error postion 
	float error_norm = sqrt(pow(pos_err_I.x, 2) + pow(pos_err_I.y, 2));

	// Find the error theta 
	float error_theta = atan2(pos_err_I.y,pos_err_I.x);

	// Output boundary
	if (error_norm > 2) error_norm = 2;
	
	// error 
	if (error_norm < 0.1) error_norm = error_theta= 0;

	// Design your controller here, you may use a simple P controller
	float P_linear = 1;
	float P_angular = 4;
	turtle_vel_msg.linear.x = P_linear * error_norm;
	turtle_vel_msg.angular.z = P_angular * error_theta;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_Pcontrol_B");
  	ros::NodeHandle n;

  	// declare publisher & subscriber

	// turtle subscriber
  	ros::Subscriber turtle_sub = n.subscribe<turtlesim::Pose>("/turtlesim/turtle1/pose", 1, turtle_cb); 

	turtle_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/turtle1/cmd_vel",1);
		
	// define turtle goal point

	float gx, gy;

	ROS_INFO("Please input (x,y). x>0,y>0");
	cout<<"desired_X:";
	cin>>gx;
	cout<<"desired_Y:";
	cin>>gy;	
	ros::spinOnce();

	body2WorldQuat(gx, gy, turtle.theta);
	
	// setting frequency as 10 Hz
  	ros::Rate loop_rate(10);
	
  	while (ros::ok()){
		
		ROS_INFO("goal x : %f \t y : %f\n",turtle_goal.x,turtle_goal.y);
    	ROS_INFO("pose x : %f \t y : %f\n",turtle.x,turtle.y);
    	ROS_INFO("pose theta: %f \n",turtle.theta);

		//Input your goal_point to your controller
		Positioncontrol(turtle_goal, turtle, turtle_twist);

		turtle_pub.publish(turtle_twist);

    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


