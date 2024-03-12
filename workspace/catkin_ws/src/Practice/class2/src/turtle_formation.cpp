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
turtlesim::Pose leader;
turtlesim::Pose follower1;
turtlesim::Pose follower2;

// goal points
geometry_msgs::Point leader_goal;
geometry_msgs::Point follower1_goal;
geometry_msgs::Point follower2_goal;

// turtle twist
geometry_msgs::Twist follower1_twist;
geometry_msgs::Twist follower2_twist;

// turtle publisher
ros::Publisher follower1_pub;
ros::Publisher follower2_pub;


bool reset;

struct XY{
    float x;
	float y;
};

struct XY pos_err_I;



// declare call back function
void leader_cb(const turtlesim::Pose::ConstPtr& msg)
{
	leader = *msg;
}

void follower_cb1(const turtlesim::Pose::ConstPtr& msg)
{
	follower1 = *msg;
}

void follower_cb2(const turtlesim::Pose::ConstPtr& msg)
{
	follower2 = *msg;
}


// transform leader frame to world frame
void leadertoworld2D(geometry_msgs::Point &follower_goal, turtlesim::Pose &leader)
{
    // Transform follower_goal from leader's frame to world frame
    float temp_x = follower_goal.x;
	float temp_y = follower_goal.y;
    follower_goal.x = cos(leader.theta) * temp_x - sin(leader.theta) * temp_y + leader.x;
    follower_goal.y = sin(leader.theta) * temp_x + cos(leader.theta) * temp_y + leader.y;
}



// rotate the world frame coordinate to body frame 
void worldtobodyQuat(float &x, float &y, float theta)
{
	/////////////// please uncomment the following code and finish it //////////////
	double w = cos(theta/2);
	double z = sin(theta/2);

	Eigen::Quaterniond q(w, 0, 0, z);
	Eigen::Quaterniond q_normalized(q.w()/q.norm(), q.x()/q.norm(), q.y()/q.norm(), q.z()/q.norm());
	Eigen::Quaterniond v(0, x, y, 0);
	Eigen::Quaterniond v_new = q_normalized.inverse()* v * q_normalized;
	x = v_new.x();
	y = v_new.y();
	std::cout << "x quat: " << x << std::endl;
	std::cout << "y quat: " << y << std::endl;
}


// P control for goal position in world frame 
void Positioncontrol(geometry_msgs::Point &goal, turtlesim::Pose &follower, geometry_msgs::Twist &vel_msg) {

	// error in inertia frame
	pos_err_I.x = goal.x - follower.x;
	pos_err_I.y = goal.y - follower.y;

	// Find the goal_point position in Body(turtlesim) frame
	worldtobodyQuat(pos_err_I.x, pos_err_I.y, follower.theta);

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
	vel_msg.linear.x = P_linear * error_norm;
	vel_msg.angular.z = P_angular * error_theta;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_formation");
  	ros::NodeHandle n;

  	// declare publisher & subscriber

	// turtle subscriber
  	ros::Subscriber leader_sub = n.subscribe<turtlesim::Pose>("/turtlesim/leader/pose", 1, leader_cb); 
  	ros::Subscriber follower_sub1 = n.subscribe<turtlesim::Pose>("/turtlesim/follower1/pose", 1, follower_cb1);
	ros::Subscriber follower_sub2 = n.subscribe<turtlesim::Pose>("/turtlesim/follower2/pose", 1, follower_cb2);

	follower1_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower1/cmd_vel", 1);
	follower2_pub = n.advertise<geometry_msgs::Twist>("/turtlesim/follower2/cmd_vel", 1);
		


	// setting frequency as 10 Hz
  	ros::Rate loop_rate(10);
	
  	while (ros::ok()){

		/*     define formation of turtle 

				follower2 >
						       leader >
				follower1 >    
		*/

        follower1_goal.x = -1;
        follower1_goal.y = -1;

		follower2_goal.x = -1;
		follower2_goal.y = 1;
		
        // rotate from leader turtle frame to world frame
        leadertoworld2D( follower1_goal, leader);
		leadertoworld2D( follower2_goal, leader);

		//Input your goal_point to your controller
    	Positioncontrol(follower1_goal, follower1, follower1_twist);
    	Positioncontrol(follower2_goal, follower2, follower2_twist);

		//Input your control input(from Pcontrol) to your plant
    	follower1_pub.publish(follower1_twist);
		follower2_pub.publish(follower2_twist);

    	ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


