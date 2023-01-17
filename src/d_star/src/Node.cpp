#include <string>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "Node.h"

Node::Node(){
    std::cout << "Constructor" << std::endl;
	parent = nullptr;
    tag = TAGS::NEW;
    h = 0;
    k = 0;
}

Node::Node(std::vector<int> point_){
    point = point_;
    parent = nullptr;
    tag = TAGS::NEW;
    h = 0;
    k = 0;
}

Node::~Node(){
    /*Node *child = NULL;
    Node *parent = this;
    // std::cout << "Destructor" << std::endl;
    for(int it = 0; it < parent->childrenNumber(); it++){
        child = parent->getChild(it);
        if(child->hasChildren()){
            child->~Node();
        }
        else{
            child->point.clear();
            child->children.clear();
            child->parent = NULL;
            //delete child;
        }
    }    
    this->point.clear();
    this->children.clear();
    this->parent = NULL;*//////////////////////////    
}

void Node::setParent(Node *theParent)
{
    parent = theParent;
}

bool Node::hasParent()
{
    return parent != nullptr;
}

Node* Node::getParent()
{
    return parent;
}

std::vector<Node*> Node::getNeighbours(const std::vector<std::vector<Node*>>& graph)
{   
    std::vector<Node*> n;

    if(point[0] > 0){

        n.push_back(graph[point[0]-1][point[1]]);
        if( point[1] > 0){
            n.push_back(graph[point[0]-1][point[1]-1]);
        }
        if( point[1] < graph[point[0]].size() - 1){
            n.push_back(graph[point[0]-1][point[1]+1]);
        }
    }

    if(point[0] < graph.size() - 1){

        n.push_back(graph[point[0]+1][point[1]]);
        if( point[1] > 0){
            n.push_back(graph[point[0]+1][point[1]-1]);
        }
        if( point[1] < graph[point[0]].size() - 1){
            n.push_back(graph[point[0]+1][point[1]+1]);
        }
    }

    if(point[1] < 0){
        n.push_back(graph[point[0]][point[1]-1]);
    }

    if(point[1] < graph[point[0]].size() - 1){
        n.push_back(graph[point[0]][point[1]+1]);
    }

    return n;
}

std::vector <int> Node::getNode() 
{
    return point;
}

void Node::setNode(int x, int y){
    point[0] = x;
    point[1] = y;
}

void Node::printNode() 
{
	std::cout << "Node: (" << point[0] << "," << point[1] << ").-k: " << k << ", h: " << h << " tag: " << tag << std::endl;
}

std::vector <std::vector <int>> Node::returnSolution(){

	std::vector <std::vector <int>> solution;
	Node *node = this;
	std::cout << "Start returning the solution" << std::endl;
	while(node->hasParent()){
		
        //node->printNode();
		if(node->hasParent()){
            node = node->getParent();
            solution.push_back(node->getNode());
        }
	};
	std::cout << "Finish returning the solution" << std::endl;
	return solution;
}


