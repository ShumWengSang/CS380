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
    float Cost = -1;
};

//Disable packing
#pragma pack(push, 1)
struct GridPosChar
{
    // We can afford to use signed as the largest map is 40x40
    char row;
    char col;

    GridPosChar() = default;
    GridPosChar(char row, char col) : row(row), col(col)
    {}

    explicit GridPosChar(GridPos const& grid) : row(grid.row), col(grid.col)
    {}

    bool operator==(const GridPosChar& rhs) const
    {
        return row == rhs.row && col == rhs.col;
    }

    bool operator!=(const GridPosChar& rhs) const
    {
        return row != rhs.row || col != rhs.col;
    }
};
#pragma pack(pop)

class Array2D
{
    Node* dataPtr;
    std::vector<Node> nodes;
    int width, height;
public:
    Node& GetNode(int x, int y);
    Node& GetNode(GridPosChar position);
    // Returns the x,y of the address passed in.
    auto GetPosition(Node* ptr);
    bool isStraight(int parentX, int parentY, int childX, int childY);

    void SetSize(int width, int height);
    void Reset();
    void Clear();
};

//Disable packing
#pragma pack(push, 1)
struct Node
{   
    float finalCost = 0;
    float givenCost = 0;
    GridPosChar parentPosition = {-1,-1};
    ListType onList = ListType::None;
};
#pragma pack(pop)

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
    GridPos GoalPos;

    // From open list find the cheapest node
    Node*& findCheapestNode();
    void initializeMap();
    float GetHeuristic(GridPos const& requester, GridPos const& goal,PathRequest const & heuristic) const;
    void ConfigureForOpenList(Node* node, GridPos gridPos, float finalCost, float gx, PathRequest& request);
    void ConfigureForClosedList(Node* node, GridPos gridPos, PathRequest& request);
    void FinalizeEndPath(PathRequest& request, Node* endNode);
    float CalcCost(int parentX, int parentY, int childX, int childY);
    void GetAllChildNodes(int x, int y, GetNodeInformation(&arr)[8]);

    // Node operator functions
    Node* GetNextNode(Node* node);
    void PlaceParentIntoPath(WaypointList& path, Node* ptr);
    void PlaceThisIntoPath(WaypointList& path, Node* ptr);
    GridPos GetPosition(Node* node);

    // Post process helper
    bool isWallInRange(int xBegin, int yBeing, int xEnd, int yEnd);
    bool Smoothing(WaypointList& path);
    bool Rubberbanding(WaypointList& path, Node* endNode);
    void NormalNodesToPath(WaypointList& path, Node* endNode);
    void RubberbandSmooth_AddNodes(WaypointList& path);

    // Returns true if it split, false if it didn't.
    bool Split(WaypointList::iterator const& a, WaypointList::iterator const& b, WaypointList& path);
    float SplitDistance = 0.0f;

    struct UsefulInformation
    {
        int MaxSize = std::numeric_limits<int>::min();
        float AvgSize = 0;

        float maxCost = std::numeric_limits<float>::min();
        float avgCost = 0;

        float numOfRounds = 0;
        void Update(Node* node, int size)
        {
            numOfRounds++;
            if (size > MaxSize) MaxSize = size;
            if (node->finalCost > maxCost) maxCost = node->finalCost;
            avgCost += node->finalCost;
            AvgSize += size;
        }

        void End()
        {
            avgCost /= (float)numOfRounds;
            AvgSize /= (float)numOfRounds;
            numOfRounds = 0;
        }

        void Print()
        {
            std::cout << "-------------------------------------" << std::endl;
            std::cout << "Max Size: " << MaxSize << std::endl;
            std::cout << "Avg Size: " << AvgSize<< std::endl;
            std::cout << "Max Cost: " << maxCost << std::endl;
            std::cout << "Avg Size: " << avgCost << std::endl;
            std::cout << "-------------------------------------" << std::endl;
        }
    };

    UsefulInformation info;
};