#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <stdio.h> 
#include <math.h>
#include <fstream>
#include <tf/transform_broadcaster.h>

using namespace std;


class Lowlevelcontrol {
	ros::NodeHandle nh_;
	ros::Publisher velocity_pub_;
	ros::Subscriber position_sub_;
	ros::Subscriber goal_sub_;
	geometry_msgs::PoseStamped Goal;
	float krho, kalpha, kbeta;
public:
	Lowlevelcontrol() {

		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &Lowlevelcontrol::positionCb, this);
		goal_sub_ = nh_.subscribe("goal", 1, &Lowlevelcontrol::goalCb, this);
		velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		
		Goal.pose.position.x = 0; //update the initial values of these variables
		Goal.pose.position.y = 0;
		kalpha = 0; // the values of these parameters should be obtained by you
		krho = 0;
		kbeta = 0;
	}

	~Lowlevelcontrol() {
	}

	void goalCb(const geometry_msgs::PoseStamped& msg) {

		std::cout << " Goal Update: "<< msg.pose.position.x << endl;
		std::cout << " Goal Update: "<< msg.pose.position.y << endl;
	//	upadte the goal
	}

	void positionCb(const nav_msgs::Odometry& msg) {

		
		float ex = msg.pose.pose.position.x - Goal.pose.position.x;
		float ey = msg.pose.pose.position.y - Goal.pose.position.y;
		float rho = sqrt(ex*ex+ey*ey);
		float beta = atan2(ey,ex) + M_PI;
		if (beta < -M_PI) beta = 2*M_PI - abs(beta);
		if (beta > M_PI) beta = -2*M_PI + beta;
		float alpha = beta - tf::getYaw(msg.pose.pose.orientation);

		std::cout << "ex: "<< ex << " ";
		std::cout << "ey: "<< ey << " ";

		std::cout << "Rho: "<< rho << " ";
		std::cout << "Alpha: "<< alpha << " ";
		std::cout << "Beta: "<< beta << endl;

		std::cout << "X: "<< msg.pose.pose.position.x << " ";
		std::cout << "Y: "<< msg.pose.pose.position.y << " ";
		std::cout << "Th: "<< tf::getYaw(msg.pose.pose.orientation) << endl;

		geometry_msgs::Twist input; //to send the velocities

		//here you have to implement the controller
		velocity_pub_.publish(input);
	}

	
};

int main(int argc, char** argv) {


	ros::init(argc, argv, "lowcontrol");
	ros::NodeHandle nh("~");
	Lowlevelcontrol llc;

	ros::spin();
	return 0;
}
