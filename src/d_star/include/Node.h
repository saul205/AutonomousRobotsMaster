#ifndef Node_H
#define Node_H

#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

enum TAGS{
    OPEN,
    CLOSED,
    NEW
};

class Node
{
    private:
        std::vector <int> point;
        Node* parent;

        int countNodesRec(Node *root, int& count);

    public:

        bool obstacle = false;
        float k, h;
        TAGS tag;

        Node();
		Node(std::vector <int> point_);
		~Node();
		bool hasNeighbours();
        void setParent(Node *parent);

        bool hasParent();

        Node* getParent();
        std::vector<Node*> getNeighbours(const std::vector<std::vector<Node*>>& graph);

		std::vector <int> getNode();
        void setNode(int x, int y);
		void printNode();

		std::vector <std::vector <int>> returnSolution();
};

#endif
