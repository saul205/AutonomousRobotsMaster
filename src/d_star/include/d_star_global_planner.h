/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

#include <time.h> 
#include "Node.h"

#ifndef D_STAR_PLANNER_CPP
#define D_STAR_PLANNER_CPP

namespace d_star_planner {

struct Cmp{
    bool operator()(const Node* lhs, const Node* rhs) const { 
        return lhs->k < rhs->k; 
    }
};

class DStarPlanner : public nav_core::BaseGlobalPlanner {
    
public:

    DStarPlanner();
    DStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    std::set<Node*, Cmp> open;

    // overridden classes from interface nav_core::BaseGlobalPlanner
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

private:

    costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
    std::string global_frame_id_;
	bool initialized_;
    ros::Publisher vis_pub;

    double max_samples_;

    double max_dist_;
    double resolution_;

    // functions to compute the plan
    bool obstacleFree(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1);
    bool computeDStar(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol);
    void getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan);


    //------------------------------------------------
    void insert(Node * node, float h);
};

};
 #endif
