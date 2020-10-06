#pragma once
#include "Misc/PathfindingDetails.hpp"
#include <vector>

enum class ListType : char
{
    None,
    OpenList,
    ClosedList,
};

struct Node;
struct GetNodeInformation
{ 
    Node* childNode;
    float Cost;
    bool Possible;
};

class Array2D
{
    Node* dataPtr;
    std::vector<Node> nodes;
    int width, height;
public:
    Node& GetNode(int x, int y);
    Node& GetNode(GridPos position);
    // Returns the x,y of the address passed in.
    auto GetPosition(Node* ptr);
    void GetAllChildNodes(int x, int y, GetNodeInformation(&arr)[8]);
    bool isStraight(int parentX, int parentY, int childX, int childY);

    void SetSize(int width, int height);
    void Reset();
    void Clear();
};
  
struct Node
{   
    float cost = 0;
    float given = 0;
    GridPos parentPosition = {-1,-1};
    ListType onList = ListType::None;
};

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */


    using NodeVector = std::vector<Node*>; 
    // Array of Nodes to represent our map
    Array2D NodeMap;
    NodeVector OpenList;
    Node* EndGoal;

    // From open list find the cheapest node
    Node*& findCheapestNode();
    void initializeMap();
    float GetHeuristic(Heuristic heuristic) const;
    void ConfigureForOpenList(Node* node, GridPos gridPos, float cost, PathRequest& request);
    void ConfigureForClosedList(Node* node, GridPos gridPos, PathRequest& request);
    void FinalizeEndPath(PathRequest& request, Node* endNode);
};