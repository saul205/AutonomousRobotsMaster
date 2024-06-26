#include <string>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "Node.h"

Node::Node(){
    std::cout << "Constructor" << std::endl;
	parent_idx = -1;
    tag = TAGS::NEW;
    h = 0;
    k = 0;
}

Node::Node(std::vector<int> point_){
    point = point_;
    parent_idx = -1;
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

bool Node::hasNeighbours()
{
    if(neighbours.size() > 0)
        return true;
    else
        return false;
}

void Node::addNeighbour(Node *neighbour, float cost){
    this->neighbours.push_back(std::pair<Node*, float>(neighbour, cost));
}

void Node::appendNeighbour(Node *neighbour, float cost)
{       
    neighbour->addNeighbour(this, cost);
    this->addNeighbour(neighbour, cost);
}

void Node::modifyNeighbourCost(int idx, float cost){
    this->neighbours[idx].second += cost;
}

void Node::setParent(Node *theParent)
{
    for(int i = 0; i < neighbours.size(); i++){
        if(neighbours[i].first == theParent){
            parent_idx = i;
            break;
        }
    }
}

void Node::setParent(int index)
{
    parent_idx = index;
}

bool Node::hasParent()
{
    if(parent_idx < 0)
        return false;

    if(neighbours[parent_idx].first != NULL)
        return true;
    else 
        return false;
}

std::pair<Node*, float> Node::getParent()
{
    if(parent_idx < 0)
        return std::pair<Node*, float>(nullptr, -1);

    return neighbours[parent_idx];
}

std::vector<std::pair<Node*, float>> Node::getNeighbours()
{   
    return neighbours;
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

void Node::printTree()
{   
    Node *child = NULL;
    
    std::cout << "Parent node: (" << point[0] << "," << point[1] << ")." << std::endl;
	
    for(int it = 0; it < neighbours.size(); it++)
    {   
        std::cout << "    Child node: (" << neighbours[it].first->point[0] << "," << neighbours[it].first->point[1] << ")." << std::endl;
    }
    for(int it = 0; it < neighbours.size(); it++)
    {   
        neighbours[it].first->printTree();
    }
}

std::vector <std::vector <int>> Node::returnSolution(){

	std::vector <std::vector <int>> solution;
	Node *node = this;
	std::cout << "Start returning the solution" << std::endl;
	while(node->hasParent()){
		
        //node->printNode();
		if(node->hasParent()){
            node = node->getParent().first;
            solution.push_back(node->getNode());
        }
			
		
	};
	std::cout << "Finish returning the solution" << std::endl;
	return solution;
}


