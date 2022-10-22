#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FollowTargetsClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
    ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget = 0; //index with the next target to reach


public:
	FollowTargetsClass(string filePath, ros::NodeHandle nh) { //in the contructor you can read the targets from the text fileh_

		nh_ = nh;

		inFile.open(filePath);
		if(inFile.is_open()){
			float x = 0, y = 0;
			char separator; 

			inFile >> x >> separator >> y >> separator;
			while(!inFile.eof()){
			
				targets.push_back(std::vector<float>{x, y});
				cout << x << y << endl;
				inFile >> x >> separator >> y >> separator;
			}
		}
		cout << "Loaded" << endl;
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
		position_sub_ = nh_.subscribe("/odom", 10, &FollowTargetsClass::positionCallback, this);
	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need(
	void positionCallback(const nav_msgs::Odometry::ConstPtr &msg){
		float ex = targets[currentTarget][0] - msg->pose.pose.position.x;
		float ey = targets[currentTarget][1] - msg->pose.pose.position.y;
		float distance = sqrt(ex * ex + ey * ey);  

		if(distance < 0.2 && currentTarget < targets.size()-1){
			sendNextTarget();
		}else{
			sendTarget(currentTarget);
		}
	}

	void sendNextTarget(){
		sendTarget(++currentTarget);
	}

	void sendTarget(int index){
		Goal.pose.position.x = targets[index][0];
		Goal.pose.position.y = targets[index][1];
		Goal.pose.position.z = 0;

		Goal.pose.orientation.x = 0;
		Goal.pose.orientation.y = 0;
		Goal.pose.orientation.z = 0;
		Goal.pose.orientation.w = 0;

		Goal.header.frame_id = "odom";
		Goal.header.stamp = ros::Time::now();

		goal_pub_.publish(Goal);
	}
};


int main(int argc, char** argv) {
	string filePath = "/home/arob/catkin_ws/src/arob_lab2/src/targets.txt";
	if(argc == 2){
		filePath = argv[1];
	}

	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh;

	FollowTargetsClass FT(filePath, nh);

	ros::spin();
	return 0;
}

