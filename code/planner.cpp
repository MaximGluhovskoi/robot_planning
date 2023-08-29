/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
#include <queue>
#include <iostream>
#include <vector>
#include <set>

// C++ code to implement f_val Queue 
// using Linked List 

//Node
struct node 
{ 
    node(int xpose, int ypose, int g_val, int h_val, int f_val, struct node* parent) :
       xpose{xpose}, ypose{ypose}, g_val{g_val}, h_val{h_val}, f_val{f_val}, parent{parent} {}
    int xpose;
    int ypose;
    int g_val; 
    int h_val;
    // Lower values indicate 
    // higher f_val 
    int f_val; // f_val = g+h
    struct node* parent;
};
  
// this is a structure which implements the
// operator overloading for the f_val queue implementation

struct compare_nodes {
    bool operator()(node* const node1, node* const node2)
    {
        // return "true" if "node1" is ordered
        // before "node2", for example:
        if (node1->f_val> node2->f_val)
        {
            return true;
        }
        else if (node1->f_val == node2->f_val && node1->h_val> node2->h_val)
        {
            return true;
        }
        return false;
    }
};

struct compare_nodes_set {
    bool operator()(node* const node1, node* const node2)
    {
        if (node1->xpose < node2->xpose)
            return true;
        else if (node1->xpose == node2->xpose && node1->ypose < node2->ypose)
            return true;
        else 
            return false;
        }
};

std::vector<node*> myPath;

int heuristic_diagonal(int robotposeX, int robotposeY, int goalposeX, int goalposeY)
{
    //using a simple diagonal heuristic:
    return fmax(abs(robotposeX - goalposeX), abs(robotposeY - goalposeY));
}

int cost_function(int heuristic, int g_val)
{
    return heuristic + g_val;
}

bool check_at_goal (node* current_node, int goalposeX, int goalposeY)
{
    if (current_node->xpose == goalposeX && current_node->ypose == goalposeY)
    {
        return true;
    }
    return false;
}

void get_neighbors (double* map, int collision_thresh, 
        int x_size, int y_size, 
        int robotposeX, int robotposeY, 
        node* current_node, int goalposeX, std::priority_queue<node*, std::vector<node*>, compare_nodes> &p_queue,
        int goalposeY, std::set<node*, compare_nodes_set> &closed_set, std::set<node*, compare_nodes_set> &opened_set)
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    //going through each neighbor:
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                int g_val = current_node->g_val+(int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                int h_val = heuristic_diagonal(newx, newy, goalposeX, goalposeY);
                int f_val = cost_function(h_val, g_val);
                node* temp_node = new node(newx, newy, g_val, h_val, f_val, current_node);
                if (closed_set.find(temp_node) == closed_set.end())
                {
                    //mexPrintf("f_val: %d\n", f_val);
                    opened_set.insert(temp_node);
                    p_queue.push(temp_node);
                }
            }
        }
    }
}

void a_star (
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    //setting up goal positions currently just final place
    //mexPrintf("starting pose (X,Y): (%d,%d)\n", robotposeX, robotposeY);
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    int startingposeX = robotposeX;
    int startingposeY = robotposeY;
    //creating p_queue:
    std::priority_queue<node*, std::vector<node*>, compare_nodes> p_queue;
    //open set:
    std::set<node*, compare_nodes_set> open_set;
    //closed set:
    std::set<node*, compare_nodes_set> closed_set; //should be empty at start
    
    //creating starting node and pushing onto p_queue:
    node* temp_node = new node(startingposeX, startingposeY, 0, heuristic_diagonal(robotposeX, robotposeY, goalposeX, goalposeY), heuristic_diagonal(robotposeX, robotposeY, goalposeX, goalposeY), NULL);
    p_queue.push(temp_node);
    open_set.insert(temp_node);
    while(p_queue.size()!=0)
    {
        //mexPrintf("size of queue: %d\n", p_queue.size());
        node* current_node = p_queue.top();
        /*while (closed_set.find(current_node)!=closed_set.end())
        {
            p_queue.pop();
            current_node = p_queue.top();
        }*/
        p_queue.pop();
        if (closed_set.find(current_node)!=closed_set.end())
        {
            continue;
        }
        //mexPrintf("current f_val: %d g_val: %d, x: %d, y: %d, h_val: %d\n", current_node->f_val, current_node->g_val, current_node->xpose, current_node->ypose, current_node->h_val);
        //remove from open add to closed
        open_set.erase(current_node);
        closed_set.insert(current_node);

        if (check_at_goal(current_node, goalposeX, goalposeY))
        {
            //return the best path following the parents of this current_node
            //mexPrintf("exited %d\n", goalposeX);
            while (current_node->parent != NULL)
            {
                myPath.push_back(current_node);
                current_node = current_node->parent;
            }
            return;
        }
        get_neighbors(map, collision_thresh, x_size, y_size, current_node->xpose, current_node->ypose, current_node, goalposeX, p_queue, goalposeY, closed_set, open_set);
        //mexPrintf("size of queue: %d\n", p_queue.size());
        //get neighbors of current_node using function (take into account threshold)
        //and add to priority queue and then continue process
    }
    //mexPrintf("exhuasted all options\n");
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    if (myPath.size() == 0 && goalposeX != robotposeX && goalposeY != robotposeY)
    {
        a_star(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, target_traj, targetposeX, targetposeY,curr_time, action_ptr);
    }
    else if (myPath.size() != 0)
    {
        node* cur_path = myPath.back();
        if (heuristic_diagonal(robotposeX,robotposeY, cur_path->xpose,cur_path->ypose)>1)
        {
            myPath.clear();
            mexPrintf("had to clear myPath most likely due to some values still being in myPath after previous run\n");
            action_ptr[0]= robotposeX;
            action_ptr[1]= robotposeY;
            return;
        }
        action_ptr[0] = cur_path->xpose;
        action_ptr[1] = cur_path->ypose;
        myPath.pop_back();
        return;
    }
    action_ptr[0]= robotposeX;
    action_ptr[1]= robotposeY;
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
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
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}