#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <set>
#include "mex.h"
#include "AStar.h"


#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

/* Input Arguments */
#define MAP_IN      prhs[0]
#define ROBOT_IN  prhs[1]
#define GOAL_IN     prhs[2]

/* Output Arguments */
#define ACTION_OUT  plhs[0]

using namespace std; 

//   --------------------------- helper for debugging -----------------------------------------
// void showpq(priority_queue <Node*, vector<Node*>, myComparator> gq) 
// { 
//     priority_queue <Node*, vector<Node*>, myComparator> g = gq; 
//     while (!g.empty()) 
//     { 
//         cout << g.top()->x << " " << g.top()->y << " " << g.top()->gVal << " " << int(g.top()->gVal+eps*g.top()->hVal) <<endl; 
//         g.pop(); 
//     } 
// }

// ------------------------------------------------------------------------------------------


AStar::AStar(){}

AStar::~AStar(){
//     delete startNode;
//     delete goalNode;
}

float AStar::getHeuristic(Node n1, Node n2){
    // implement diagonal distance for 8-connected grids
    double h;
    h = max(abs(n1.x-n2.x), abs(n1.y-n2.y));
    return h;
}


float AStar::getCost(Node n1, Node n2){
    return sqrt(pow(n2.x-n1.x,2)+pow(n2.y-n1.y,2));
//     return 1;
}

bool AStar::isValid(double* map, int x, int y, int x_size, int y_size){
    if((x+1>0) && (x+1 <= x_size) && (y+1>0) && (y+1 <= y_size)){
        if ((int)map[GETMAPINDEX(x+1,y+1,x_size,y_size)] != 1){
            return true;
        } 
        else {
            return false;
        } 
    } else{
        return false;
    }
  
}

bool AStar::isFrontier(double* map, int x, int y, int x_size, int y_size){
    bool isFrontier = false;
//     if(exploredList.find(make_pair(x, y))==exploredList.end()){
//         isFrontier = true;
//     }
    
    if ((int)map[GETMAPINDEX(x+1,y+1,x_size,y_size)]==0){
        isFrontier = true;
    }            
    return isFrontier;
}

int myrandom (int i) { return std::rand()%i;}

void AStar::plan(
        double* map,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int goalposeX,
        int goalposeY,
        char *p_actionX,
        char *p_actionY)
{   
        
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    robotposeX = robotposeX-1;
    robotposeY = robotposeY-1;
    
    Node* virtualGoal = new Node;
    virtualGoal->x = goalposeX; // change
    virtualGoal->y = goalposeY; // change
    
    virtualGoal->gVal = FLT_MAX;
    virtualGoal->hVal = 0;
    virtualGoal->isExpanded = false;
    
    Node* startNode = new Node;
    startNode->x = robotposeX;
    startNode->y = robotposeY;
    startNode->gVal = 0;
    startNode->hVal = getHeuristic(*startNode, *virtualGoal);
    startNode->isExpanded = false;
    startNode->bp = NULL;
            
    priority_queue <Node, vector<Node>, myComparator> openList;
    openList.push(*startNode);
    
    set<pair<int,int>> nodesInOpen;
        
    while(virtualGoal->isExpanded==false && openList.empty()==false){
        Node S = openList.top();
        Node* currNode = new Node;
        currNode->x = S.x;
        currNode->y = S.y;
        currNode->gVal = S.gVal;
        currNode->hVal = S.hVal;
        currNode->bp = S.bp;
        currNode->isExpanded = true;
        
        if(currNode->hVal == 0){
            virtualGoal->isExpanded = true;
            virtualGoal->bp = S.bp;
            break;
        }
        openList.pop();
                        
        // if the expanded node is frontier
        if(isFrontier(map, currNode->x,currNode->y,x_size,y_size)){
            
            Node* successor = new Node;
            successor->x = virtualGoal->x;
            successor->y = virtualGoal->y;
            successor->gVal = FLT_MAX;
            successor->hVal = 0;
            
            float val = currNode->gVal+1;
           // float val = currNode->gVal+getCost(*successor, *virtualGoal);
            if(successor->gVal>val){
                successor->gVal = val;
                successor->bp = currNode;
                openList.push(*successor);
            }
        }
        else{
            for (int dir=0;dir<NUMOFDIRS;dir++){
                int xInd = currNode->x+dX[dir];
                int yInd = currNode->y+dY[dir];
                                
                if (xInd>=0 && yInd>=0 && xInd < x_size && yInd < y_size){
                    Node* successor = new Node;
                    successor->x = xInd;
                    successor->y = yInd;
                    successor->gVal = FLT_MAX;
                    successor->hVal = getHeuristic(*successor, *virtualGoal);
                    if(isValid(map,successor->x,successor->y,x_size,y_size) || isFrontier(map, successor->x,successor->y,x_size,y_size)){
                        float val = currNode->gVal+getCost(*currNode, *successor);
                        if(successor->gVal>val){
                            successor->gVal = val;
                            successor->bp = currNode;
                            pair<int,int> coord(successor->x, successor->y);
                            if(nodesInOpen.find(coord) == nodesInOpen.end()){
                                openList.push(*successor);
                                nodesInOpen.insert(coord);
                            }
                        }
                    }
                }
            }
        }
    }

    // Backtracking
    Node* last_node = virtualGoal->bp;
    if (openList.empty()){
        *p_actionX = 0;
        *p_actionY = 0;
    } else{
        while (last_node->x!= robotposeX || last_node->y!= robotposeY){
            *p_actionX = last_node->x - robotposeX;
            *p_actionY = last_node->y - robotposeY;
            last_node = last_node->bp;
        }
    }
}


// Create MEX function
mxArray * getMexArray(const std::vector<double>& v){
    mxArray * mx = mxCreateDoubleMatrix(1,v.size(), mxREAL);
    std::copy(v.begin(), v.end(), mxGetPr(mx));
    return mx;
}


void mexFunction( int nlhs, mxArray *plhs[], 
      int nrhs, const mxArray*prhs[] )
{ 
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
      mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
      mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
      mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 2){
      mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 2.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    int goalposeX = (int)goalposeV[0];
    int goalposeY = (int)goalposeV[1];
        
//     cout << "------------- set up info --------------" <<endl;
//     cout << "Grid size: " << x_size << "x" << y_size <<endl;
//     cout << "Eps initial: " << eps <<endl;
//     cout << "----------------------------------------" <<endl;
    

    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxINT8_CLASS, mxREAL); 
    char* action_ptr = (char*)  mxGetPr(ACTION_OUT);
            
    /* Do the actual planning in a subroutine */
    AStar plannerAStar;
    plannerAStar.plan(map, x_size, y_size, robotposeX, robotposeY, goalposeX, goalposeY, &action_ptr[0], &action_ptr[1]);

    return;
}
