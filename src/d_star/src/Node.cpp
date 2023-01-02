#include <string>
#include <vector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "d_star/include/Node.h"

Node::Node(){
    std::cout << "Constructor" << std::endl;
	parent = std::pair<Node*, int>(NULL, -1);
}

Node::Node(std::vector <int> point_){
    // std::cout << "Constructor" << std::endl;
    // std::cout << "point_:" << point_[0] << ", " << point_[1] << std::endl;
    point = point_;
    parent = std::pair<Node*, int>(NULL, -1);
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

void Node::addNeighbour(Node *neighbour, int cost){
    this->neighbours.push_back(std::pair<Node*, int>(neighbour, cost));
}

void Node::appendNeighbour(Node *neighbour, int cost)
{       
    neighbour->addNeighbour(this, cost);
    this->addNeighbour(neighbour, cost);
}

void Node::setParent(Node *theParent, int cost)
{
    parent = std::pair<Node*, int>(theParent, cost);
}

bool Node::hasParent()
{
    if(parent.first != NULL)
        return true;
    else 
        return false;
}

std::pair<Node*, int> Node::getParent()
{
    return parent;
}

std::vector<std::pair<Node*, int>> Node::getNeighbours()
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
	std::cout << "Node: (" << point[0] << "," << point[1] << ")." << std::endl;
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
	//std::cout << "Start returning the solution" << std::endl;
	while(node->hasParent()){
		solution.push_back(node->getNode());
        //node->printNode();
		if(node->hasParent())
			node = node->getParent().first;
		
	};
	//std::cout << "Finish returning the solution" << std::endl;
	return solution;
}


