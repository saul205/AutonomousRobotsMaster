#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include "d_star_global_planner.h"
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(d_star_planner::DStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace d_star_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

DStarPlanner::DStarPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

DStarPlanner::DStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void DStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

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
        plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);

        current_plan = false;
        current_path_value = 0;

        initializeTree();

        initialized_ = true;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool DStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
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
    bool computed = computeDStar(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
        publishPlan(plan);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

int distance(Node* node1, Node* node2){
    std::vector <int> pos1 = node1->getNode();	
    std::vector <int> pos2 = node2->getNode();	
    return sqrt((pos1[0]-pos2[0])*(pos1[0]-pos2[0]) + (pos1[1]-pos2[1])*(pos1[1]-pos2[1]));
}

bool DStarPlanner::computeDStar(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    
    int width = costmap_->getSizeInCellsX();
    int height = costmap_->getSizeInCellsY();
    sol.clear();

    if(start[0] == goal[0] && start[1] == goal[1]){
        cout << "Global path goal reached" << endl;
        current_path_value = 0;
        return false;
    }

    open.clear();
    open.push_front(graph[goal[0] * height + goal[1]]);

    Node* start_node = graph[start[0] * height + start[1]];

    while(start_node->tag != TAGS::CLOSED && current_path_value >= 0){
        current_path_value = process_state();
        //cout << "Current path value "  << current_path_value << endl;
    }

    if(start_node->tag == TAGS::NEW){
        cout << "No path" << endl;
        return false;
    }

    Node* actual = start_node;

    while(actual != graph[goal[0] * height + goal[1]]){

        /*
        current_path_value = cost_path(actual);
        for(std::pair<Node*, int> st : actual->getNeighbours()){
            vector<int> point = st.first->getNode();
            if(st.second < obstacle_cost && costmap_->getCost(point[0], point[1]) != costmap_2d::FREE_SPACE){
                modify_cost(st.first);
                cout << "Modified cost" << endl;
            }
        }*/
        
        current_path_value = actual->h;
        vector<int> point = actual->getNode();
        if(costmap_->getCost(point[0], point[1]) != costmap_2d::FREE_SPACE){
            modify_cost(actual);
        }
        
        
        while(actual->h > current_path_value && current_path_value >= 0){
            current_path_value = process_state();
        }

        actual = actual->getParent().first;
    }

    if(actual->h < obstacle_cost){
        sol = start_node->returnSolution();
        return true;
    }
    else{
        return false;
    }
    
}

bool DStarPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
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

void DStarPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.begin(); it != sol.end(); it++){
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

void DStarPlanner::insert(Node* node, float h){

    if(node->tag == TAGS::NEW){
        node->k = h;
    }else if(node->tag == TAGS::OPEN){
        node->k = min(node->k, h);
    }else{
        node->k = min(node->h, h);
    }
    
    node->h = h;
    node->tag = TAGS::OPEN;

    bool in = false;
    for(auto it = open.begin(); it != open.end(); it++){
        if((*it) == node){
            in = true;
        }
    }

    if(!in)
        open.push_front(node);

    open.sort([ ]( const Node* lhs, const Node* rhs )
        {
        return lhs->k < rhs->k;
        });

    open.unique();
/*
    for(auto it = open.begin(); it != open.end(); it++){
        (*it)->printNode();
    }

    cin.get();
    */
}

float DStarPlanner::process_state(){

    if(open.empty()){
        return -1;
    }

    Node* x = min_state();
    if(x->k < x->h){
        int i = 0;
        for(std::pair<Node*, float> st : x->getNeighbours()){
            if(st.first->tag != TAGS::NEW && st.first->h <= x->k && x->h > st.first->h + st.second){
                x->setParent(i);
                x->h = st.first->h + st.second;
            }

            i++;
        }
    }
    if(x->k == x->h){
        int i = 0;
        for(std::pair<Node*, float> st : x->getNeighbours()){
            if(st.first->tag == TAGS::NEW 
                || ( st.first->getParent().first == x && st.first->h != x->h + st.second )
                || ( st.first->getParent().first != x && st.first->h > x->h + st.second )){
                st.first->setParent(x);
                insert(st.first, x->h + st.second);
            }

            i++;
        }
    }
    else{
        int i = 0;
        for(std::pair<Node*, float> st : x->getNeighbours()){

            if(st.first->tag == TAGS::NEW 
                || ( st.first->getParent().first == x && st.first->h != x->h + st.second )){
                st.first->setParent(x);
                insert(st.first, x->h + st.second);
            }
            else{
                
                if( st.first->getParent().first != x && st.first->h > x->h + st.second && x->tag == TAGS::CLOSED){
                    insert(x, x->h);
                }
                else if(st.first->getParent().first != x && x->h > st.first->h + st.second && x->tag == TAGS::CLOSED && st.first->h > x->k)
                {
                    insert(st.first, st.first->h);
                }

            }

            i++;
        }
    }

    return min_val();
}

Node* DStarPlanner::min_state(){
    Node* aux = nullptr;
    if(open.empty()){
        return aux;
    }

    aux = open.front();
    open.pop_front();
    aux->tag = TAGS::CLOSED;
    return aux;
}

float DStarPlanner::min_val(){
    if(open.empty()){
        return -1;
    }

    return (*open.begin())->k;
}
/*
float DStarPlanner::cost_path(Node* start){
    Node* aux = start;
    float cost = 0.f;
    while(aux->getParent().first != NULL && aux->getParent().first.getParent().first != aux){
        cost += aux->getParent().second;
        aux = aux->getParent().first;
    }
    return cost;
}
*/
void DStarPlanner::modify_cost(Node* X){
    X->h += obstacle_cost;
    vector<std::pair<Node*, float>> neighbours = X->getNeighbours();
    for(int i = 0; i < neighbours.size(); i++){
        X->modifyNeighbourCost(i, obstacle_cost);

        vector<std::pair<Node*, float>> neighbours_n = neighbours[i].first->getNeighbours();
        for(int j = 0; j < neighbours_n.size(); j++){
            if(neighbours_n[j].first == X){
                neighbours[i].first->modifyNeighbourCost(j, obstacle_cost);
                break;
            }
        }
        
        if(neighbours[i].first->tag == CLOSED){
            insert(neighbours[i].first, neighbours[i].first->h);
        }
    }
}

void DStarPlanner::initializeTree(){
    int width = costmap_->getSizeInCellsX();
    int height = costmap_->getSizeInCellsY();

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            Node* n = new Node(std::vector<int>{i, j});
            graph.push_back(n);
        }
    }

    // for(int i = 0; i < width; i++){
    //     for(int j = 0; j < height; j++){
    //         graph[i * height + j]->printNode();
    //     }
    // }

    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){

            if(i > 0){

                graph[i * height + j]->addNeighbour(graph[(i - 1) * height + j], 1 + computeExtraCost(i, j, i-1, j));
                if( j > 0){
                    graph[i * height + j]->addNeighbour(graph[(i-1) * height + j - 1], sqrt(2) + computeExtraCost(i, j, i-1, j-1));
                }
                if( j < height - 1){
                    graph[i * height + j]->addNeighbour(graph[(i-1) * height + j + 1], sqrt(2)+ computeExtraCost(i, j, i-1, j+1));
                }
            }

            if( j > 0){
                    graph[i * height + j]->addNeighbour(graph[(i) * height + j - 1], 1+ computeExtraCost(i, j, i, j-1));
            }

            if(i < width - 1){
                graph[i * height + j]->addNeighbour(graph[(i+1) * height + j], 1+ computeExtraCost(i, j, i+1, j));
                if( j > 0){
                    graph[i * height + j]->addNeighbour(graph[(i+1) * height + j - 1], sqrt(2)+ computeExtraCost(i, j, i+1, j-1));
                }
                if( j < height - 1){
                    graph[i * height + j]->addNeighbour(graph[(i+1) * height + j + 1], sqrt(2)+ computeExtraCost(i, j, i+1, j+1));
                }
            }

            if(j < height - 1){
                    graph[i * height + j]->addNeighbour(graph[(i) * height + j + 1], 1+ computeExtraCost(i, j, i, j+1));
            }
        }
    }

    // cout << width << ", " << height << endl;
    // cin.get();
    // for(int j = 0; j < height; j+=2){
    //     for(int i = 0; i < height; i+=2){
    //         if(costmap_->getCost(graph[i * height + j]->getNode()[0], graph[i * height + j]->getNode()[1]) != costmap_2d::FREE_SPACE){
    //             cout << "x";
    //         }
    //         else{
    //             cout << "_";
    //         }
    //     }
    //     cout << endl;
    // }

    // cin.get();
}

float DStarPlanner::computeExtraCost(int x, int y, int x2, int y2){
    if(costmap_->getCost(x, y) != costmap_2d::FREE_SPACE || costmap_->getCost(x2, y2) != costmap_2d::FREE_SPACE){
        return obstacle_cost;
    }

    return 0;
}

void DStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = global_frame_id_;
    gui_path.poses.resize(path.size());
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

};
