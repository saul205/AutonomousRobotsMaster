#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        nh.param("maxsamples", max_samples_, 0.0);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/3.0);  //or any other distance within local costmap

        nh_global.param("resolution", resolution_, 0.05);

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

int distance(TreeNode* node1, TreeNode* node2){
    std::vector <int> pos1 = node1->getNode();	
    std::vector <int> pos2 = node2->getNode();	
    return sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]));
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    bool finished = false;

    //Initialize random number generator
    srand(time(NULL));
        
    // Initialize the tree with the starting point in map coordinates
    TreeNode *itr_node = new TreeNode(start);
    TreeNode *goal_node = new TreeNode(goal);

    int n_samples = 0;

    double real_dist = max_dist_ / resolution_;

    int dist = distance(itr_node, goal_node);
    bool reachable = obstacleFree(start[0], start[1], goal[0], goal[1]);
    reachable = reachable && dist <= real_dist;
    while(!reachable){
        int sampleX = rand() % costmap_->getSizeInCellsX();
        int sampleY = rand() % costmap_->getSizeInCellsY();
        while(costmap_->getCost(sampleX, sampleY) != costmap_2d::FREE_SPACE){ //costmap_->getCost(sampleX, sampleY) != costmap_2d::FREE_SPACE
            sampleX = rand() % costmap_->getSizeInCellsX();
            sampleY = rand() % costmap_->getSizeInCellsY();
        }

        //cout << "Random sample " << sampleX << " " << sampleY << endl;

        TreeNode* new_node = new TreeNode(std::vector<int>{sampleX, sampleY});
        //cout << "New Node: " << endl;
        //new_node->printNode();
        TreeNode* near_node = new_node->neast(itr_node);

        //cout << "Near Node: " << endl;
        //near_node->printNode();

        dist = distance(new_node, near_node);
        if(dist > real_dist){
            continue;
        }

        if(obstacleFree(new_node->getNode()[0], new_node->getNode()[1]
                            , near_node->getNode()[0], near_node->getNode()[1])){
            near_node->appendChild(new_node);
            visualization_msgs::Marker marker;
            marker.header.frame_id = global_frame_id_;
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = n_samples;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            costmap_->mapToWorld((unsigned int)new_node->getNode()[0], (unsigned int)new_node->getNode()[1], 
                            marker.pose.position.x, marker.pose.position.y);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            vis_pub.publish(marker);

            marker.header.frame_id = global_frame_id_;
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = n_samples+1;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            geometry_msgs::Point a;
            a.z = 0;
            costmap_->mapToWorld((unsigned int)new_node->getNode()[0], (unsigned int)new_node->getNode()[1], 
                            a.x, a.y);
            geometry_msgs::Point b;
            b.z = 0;
            costmap_->mapToWorld((unsigned int)near_node->getNode()[0], (unsigned int)near_node->getNode()[1], 
                            b.x, b.y);
            marker.points = vector<geometry_msgs::Point>{a, b};
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            vis_pub.publish(marker);

            //cin.get();
  
            if(new_node->hasParent()){
                dist = distance(new_node, goal_node);
                reachable = obstacleFree(new_node->getNode()[0], new_node->getNode()[1], goal[0], goal[1]);
                reachable = reachable && dist <= real_dist;
                if(reachable){
                    //goal_node->printNode();
                    sol = new_node->returnSolution();
                }   
            }
        }

        n_samples += 2;
    }
    
    itr_node->~TreeNode();

    cout << sol.size() << "  " << reachable << endl;

    return reachable;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

    }
}

};
