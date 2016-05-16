#include "stlastar.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>

using namespace std;

char map[100][100];                 //The map that holds the values X or O where X is an invalid position and O is a valid position
unsigned int Total_Nodes;

class MapSearchNode {               //The class for each valid position in the map
public:
    int x,y;                        //The position of each Node

    MapSearchNode(){                //The Clear the position constructor of the class
        x=0;
        y=0;
    }

    MapSearchNode(int newX,int newY){   //Constructor to initialize the position in the map of each node we create
        x = newX;
        y = newY;
    }

    float GoalDistanceEstimate(MapSearchNode &nodeGoal); //Estimates the distance between the node and the goal
    bool IsGoal(MapSearchNode &nodeGoal);                //Returns true if this node is the goal
    bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node); //Builds every one node that is a valid option to choose from this node
    float GetCost(MapSearchNode &succesor);              //Returns the cost of each move
    bool IsSameState(MapSearchNode &rhs);                //Returns true if the provided state is the same as this
    pair<int,int> returnNodeInfo();                      //Returns the inforamtion for this Node
};

bool MapSearchNode::IsSameState(MapSearchNode &rhs){    //Returns true if the given state is the same as this node

    if ((x == rhs.x)&&(y == rhs.y)) return true;
    else return false;
}

pair<int,int> MapSearchNode::returnNodeInfo(){          //Returns a pair which holds the information of the position of the nodes

    pair<int,int> temp;
    temp.first = x;
    temp.second = y;
    return temp;
}
/*
float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal ) {   //This is the euristic function to estimate the distance of this node to the goal state

    return pow(x + nodeGoal.x, 2) + pow(y + nodeGoal.y, 2);
}
*/
float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal ) {   //This is the euristic function to estimate the distance of this node to the goal state

    return sqrt(pow(x - nodeGoal.x, 2) + pow(y - nodeGoal.y, 2));
}


bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal){                    //Returns true if this node is the goal state

    if((x == nodeGoal.x)&&(y == nodeGoal.y)) return true;
    else return false;
}

bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node) {

    int parent_x = -1;
    int parent_y = -1;
    if ( parent_node ) {
	parent_x = parent_node->x;
	parent_y = parent_node->y;
    }

    MapSearchNode NewNode;
                                                                                //Check every possible node we can choose and if it is a valid position then add this node to the successors of this node
    if ( map[x-1][y] == 'O' && !( (parent_x == x-1) && (parent_y == y))) {      //Also if this is a valid successor then increment the amount of the nodes
	NewNode = MapSearchNode(x-1, y);
	astarsearch->AddSuccessor( NewNode );
	Total_Nodes++;
    }
    if (map[x][y-1] == 'O' && !( (parent_x == x) && (parent_y == y-1))) {
	NewNode = MapSearchNode(x, y-1);
	astarsearch->AddSuccessor( NewNode);
	Total_Nodes++;
    }
       if ( map[x+1][y] == 'O' && !( (parent_x == x+1) && (parent_y == y))) {
	NewNode = MapSearchNode(x+1, y);
	astarsearch->AddSuccessor( NewNode );
	Total_Nodes++;
    }
    if (map[x][y+1] == 'O' && !( (parent_x == x) && (parent_y == y+1))) {
	NewNode = MapSearchNode(x, y+1);
	astarsearch->AddSuccessor( NewNode);
	Total_Nodes++;
    }
    return true;
}

float MapSearchNode::GetCost(MapSearchNode &successor){
                                                                                // 1 cost for the valid position and 10 cost for the invalid
    if (map[x][y] == 'O') return 1.0;
    else return 10.0;
}

pair<int,int> position1,position2,goal;

int main(int argc,char * argv[]) {

    int i,j;
    int N,M;
    //ios::sync_with_stdio(false);


    //Read the input from the file given
    if (argc !=2){
        cout << "Please give the file to read from in the commande line..." << endl;
        return 1;
    }
    std::ifstream infile(argv[1]);
    infile >> M >> N;
    infile >> position1.second >> position1.first;
    infile >> position2.second >> position2.first;
    infile >> goal.second >> goal.first;

    for (i=0; i<N; i++){
        for (j=0; j<M; j++){
            infile >> map[i][j];
        }
    }


    //Print the input
    cout << "Map size: " << M << " "<< N << endl;
    cout << "First robot position:" << position1.first << " " << position1.second << endl;
    cout << "Second robot position:" << position2.first << " " << position2.second << endl;
    cout << "Goal position:" << goal.first << " " << goal.second << endl;
    map[position1.first][position1.second] = 'A';
    map[position2.first][position2.second] = 'A';
    cout << "Map:" << endl;
    for (i=0; i<N; i++){
        for(j=0; j<M; j++){
            cout << map[i][j] << " " ;
        }
        cout << endl;
    }
    cout << endl;

    //Implement 1 A star for robot 1
    AStarSearch<MapSearchNode> astarsearch_1;
    unsigned int SearchCount = 0;
    const unsigned int NumSearches = 1;  //Search only 1 time

    while (SearchCount < NumSearches) {
        MapSearchNode nodeStart;        //Initialize the starting node
        nodeStart.x = position1.first;
        nodeStart.y = position1.second;

        MapSearchNode nodeEnd;          //Initialize the goal state
        nodeEnd.x = goal.first;
        nodeEnd.y = goal.second;

        astarsearch_1.SetStartAndGoalStates(nodeStart,nodeEnd); //In the A* star search

        unsigned int SearchState1;
        unsigned int SearchSteps = 0;

        do {                                                    //Keep searching while the state is SEARCHING _TATE
            SearchState1 = astarsearch_1.SearchStep();
            SearchSteps++;

        } while(SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

        if (SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) cout << "Search Suceeded for robot 1.\n";
            else if (SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
                cout << "Search Failed for robot 1.\n";
                return 1;
                }

        cout << "Total search steps for robot 1: " << SearchSteps << endl;
        SearchCount ++;
        astarsearch_1.EnsureMemoryFreed();
    }


    //Implement 1 A star for robot 2
    AStarSearch<MapSearchNode> astarsearch_2;
    SearchCount = 0;

    while (SearchCount < NumSearches) {
        MapSearchNode nodeStart;        //Initialize the starting node
        nodeStart.x = position2.first;
        nodeStart.y = position2.second;

        MapSearchNode nodeEnd;          //Initialize the goal state
        nodeEnd.x = goal.first;
        nodeEnd.y = goal.second;

        astarsearch_2.SetStartAndGoalStates(nodeStart,nodeEnd);

        unsigned int SearchState1;
        unsigned int SearchSteps = 0;

        do {
            SearchState1 = astarsearch_2.SearchStep();
            SearchSteps++;

        } while(SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

        if (SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED) cout << "Search Suceeded for robot 2.\n";
            else if (SearchState1 == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED) {
                cout << "Search Failed for robot 2.\n";
                return 1;
                }

        cout << "Total search steps for robot 2: " << SearchSteps << endl;
        SearchCount ++;
        astarsearch_2.EnsureMemoryFreed();
    }

    //Output the list of nodes into vectors
    vector< pair<int,int> > Robot1;
    MapSearchNode * node_1 = astarsearch_1.GetSolutionStart();
    Robot1.push_back(node_1->returnNodeInfo());
    for(;;){
        node_1 = astarsearch_1.GetSolutionNext();
        if (!node_1) break;
        Robot1.push_back(node_1->returnNodeInfo());
    }

    vector< pair<int,int> >Robot2;
    MapSearchNode * node_2 = astarsearch_2.GetSolutionStart();
    Robot2.push_back(node_2->returnNodeInfo());
    for(;;){
        node_2 = astarsearch_2.GetSolutionNext();
        if (!node_2) break;
        Robot2.push_back(node_2->returnNodeInfo());
    }

    //Print the output
    unsigned int iterations, step, tempStep;

    iterations = min(Robot1.size(),Robot2.size());
    cout << "Solutions Paths: " << endl;
    for (step = 0; step < iterations; step++){
        if ((Robot1[step].first == Robot2[step].first) && (Robot1[step].second == Robot2[step].second)) break;
        cout << "Robot 1 went to :" << Robot1[step].first << " " << Robot1[step].second;
        cout << "\t\tRobot 2 went to :" << Robot2[step].first << " " << Robot2[step].second << endl;
    }

    if (step == iterations) {
        if (iterations == Robot1.size()){
            for (tempStep = step; tempStep < Robot2.size()-1 ; tempStep++){
                cout << "\t\t\t\tRobot 2 went to :" << Robot2[tempStep].first << "  " << Robot2[tempStep].second << endl;
            }
        } else if (iterations == Robot2.size()){
            for (tempStep = step; tempStep < Robot1.size()-1 ; tempStep++){
                cout << "Robot 1  went to  :" << Robot1[tempStep].first << " " << Robot1[tempStep].second << endl;
            }
        }
    } else {
        //Make a choice on which robot to stall and which to move. I made that the first will move first
        cout << "Robot 1  went to  :" << Robot1[step].first << " " << Robot2[step].second;
        cout << "\t\tRobot 2 has to stall in order not to crash *** Conflict ***" << endl;
        for (tempStep = step+1; tempStep < iterations; tempStep++){
            cout << "Robot 1 went to  :" << Robot1[tempStep].first << " " << Robot1[tempStep].second;
            cout << "\t\tRobot 2  went to  :" << Robot2[tempStep-1].first << " " << Robot2[tempStep-1].second << endl;
        }
        for (step = tempStep; step < Robot2.size(); step++){
            cout << "\t\t\t\tRobot 2  went to  :" << Robot2[step].first << "  " << Robot2[step].second << endl;
        }
    }

    cout <<"Total nodes created: " << Total_Nodes << endl;
    return 0;
}


























