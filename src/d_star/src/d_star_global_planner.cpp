#include <pluginlib/class_list_macros.h>
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
        current_plan = false;
        current_path_value = 0;
        return true;
    }

    if(!current_plan){
        open.clear();

        open.insert(graph[goal[1] * width + goal[0]]);

        Node* start_node = graph[start[1] * width + start[0]];

        while(start_node->tag != TAGS::CLOSED && current_path_value >= 0){
            current_path_value = process_state();
        }

        if(start_node->tag == TAGS::NEW){
            cout << "No path" << endl;
            return false;
        }else
            sol = start_node->returnSolution();

        current_plan = true;
    }
    else{

        Node* start_node = graph[start[1] * width + start[0]];
        
        for(std::pair<Node*, int> st : start_node->getNeighbours()){
            vector<int> point = st.first->getNode();
            if(costmap_->getCost(point[0], point[1]) != costmap_2d::FREE_SPACE){
                modify_cost(st.first);
            }
        }

        while(cost_path(start_node) > current_path_value >= 0)
            current_path_value = process_state();

        if(current_path_value < 10000000){
            sol = start_node->returnSolution();
            return true;
        }
        else{
            return false;
        }
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
        
    open.insert(node);
}

float DStarPlanner::process_state(){

    if(open.empty()){
        return -1;
    }

    Node* x = min_state();
    if(x->k < x->h){
        int i = 0;
        for(std::pair<Node*, int> st : x->getNeighbours()){
            if(st.first->tag != TAGS::NEW && st.first->h <= x->k && x->h > st.first->h + st.second){
                x->setParent(i);
                x->h = st.first->h + st.second;
            }

            i++;
        }
    }

    if(x->k == x->h){
        int i = 0;
        for(std::pair<Node*, int> st : x->getNeighbours()){
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
        for(std::pair<Node*, int> st : x->getNeighbours()){

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

/*def process_state():
    if len(open) <= 0: return -1

    x = min_state()
    if x.k < x.h:

        for st in getNeighbors(x.pos):

            if st.tag != 'NEW' and st.h <= x.k and x.h > st.h + cost(x, st):
                x.b = st
                x.h = st.h + cost(x, st)

    if x.k == x.h:

        for st in getNeighbors(x.pos):

            if st.tag == 'NEW' or \
                    (st.b == x and st.h != x.h + cost(x, st)) or \
                    (st.b != x and st.h > x.h + cost(x, st)):
                st.b = x
                insert(st, x.h + cost(x, st))

    else:

        for st in getNeighbors(x.pos):

            if st.tag == 'NEW' or \
                    (st.b == x and st.h != x.h + cost(x, st)):

                st.b = x
                insert(st, x.h + cost(x, st))

            else:

                if st.b != x and st.h > x.h + cost(x, st) and x.tag == 'CLOSED':
                    insert(x, x.h)

                else:

                    if st.b != x and x.h > st.h + cost(x, st) and st.tag == 'CLOSED' and st.h > x.k:
                        insert(st, st.h)

    return min_val()*/

    Node* DStarPlanner::min_state(){
        Node* aux = nullptr;
        if(open.size() <= 0){
            return aux;
        }

        aux = (*open.begin());
        
        open.erase(open.begin());
        return aux;
    }

    float DStarPlanner::min_val(){
        if(open.size() <= 0){
            return -1;
        }

        return (*open.begin())->k;
    }

    float DStarPlanner::cost_path(Node* start){
        Node* aux = start;
        float cost = 0.f;
        while(aux->getParent().first != NULL/*&& aux->getParent().first.getParent().first != aux*/){
            cost += aux->getParent().second;
            aux = aux->getParent().first;
        }
        return cost;
    }

    float DStarPlanner::modify_cost(Node* X){
        
        vector<std::pair<Node*, int>> neighbours = X->getNeighbours();
        for(int i = 0; i < neighbours.size(); i++){
            neighbours[i].second += 10000000;

            vector<std::pair<Node*, int>> neighbours_n = neighbours[i].first->getNeighbours();
            for(int j = 0; j < neighbours_n.size(); j++){
                if(neighbours_n[j].first == X){
                    neighbours_n[i].second += 10000000;
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
                graph.push_back(new Node(std::vector<int>(i, j)));
            }
        }

        for(int i = 0; i < width; i++){
            for(int j = 0; j < height; j++){

                if(i > 0){

                    graph[j * width + i]->addNeighbour(graph[j * width + i - 1], 1);
                    if( j > 0){
                        graph[j * width + i]->addNeighbour(graph[(j-1) * width + i - 1], sqrt(2));
                    }
                    if( j < height - 1){
                        graph[j * width + i]->addNeighbour(graph[(j+1) * width + i - 1], sqrt(2));
                    }
                }

                if( j > 0){
                     graph[j * width + i]->addNeighbour(graph[(j-1) * width + i], 1);
                }

                if(i < width - 1){
                    graph[j * width + i]->addNeighbour(graph[j * width + i + 1], 1);
                    if( j > 0){
                        graph[j * width + i]->addNeighbour(graph[(j-1) * width + i + 1], sqrt(2));
                    }
                    if( j < height - 1){
                        graph[j * width + i]->addNeighbour(graph[(j+1) * width + i + 1], sqrt(2));
                    }
                }

                if(j < height - 1){
                     graph[j * width + i]->addNeighbour(graph[(j+1) * width + i], 1);
                }
            }
        }
    }

};
