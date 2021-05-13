#include <stdio.h>
#include <iostream>
#include <math.h>
#include "mex.h"
#include <vector>
#include <map>
#include <queue>

using namespace std; 

struct Node
{
    int x;
    int y;
    bool isExpanded;
    float gVal;
    float hVal;
    float vVal;
    struct Node *bp;
};

class myComparator
{
public:
    int operator() (const Node& n1, const Node& n2)
    {
        double eps = 0.2;
        return n1.gVal+eps*n1.hVal > n2.gVal+eps*n2.hVal;
//        return n1.gVal > n2.gVal;
    }
};

class AStar{

public:
    double* map;
    int x_size, y_size;
    int robotposeX, robotposeY;
    int goalposeX, goalposeY;

    AStar();
    ~AStar();
    float getHeuristic(Node n1, Node n2);
    float getCost(Node n1, Node n2);
    bool isValid(double* map, int x, int y, int x_size, int y_size);
    bool isFrontier(double* map, int x, int y, int x_size, int y_size);
    void plan(double* map,int x_size,int y_size,int robotposeX,int robotposeY,int goalposeX,int goalposeY, char *p_actionX, char *p_actionY);


private:
    priority_queue <Node*, vector<Node*>, myComparator> openList;
};

