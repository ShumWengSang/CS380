#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"


static const float WallCost = 9999;

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

auto Array2D::GetPosition(Node* ptr)
{
    long long remainder = ptr - const_cast<Node*>(nodes.data());

    if ((long)width * height <= remainder)
    {
        DebugBreak();
    }

    long long x = 0, y = 0;
    x = remainder / width;
    y = remainder - (width * x);
    return std::make_tuple((int)x, (int)y);
}


bool AStarPather::initialize()
{
    // handle any one-time setup requirements you have

    /*
        If you want to do any map-preprocessing, you'll need to listen
        for the map change message.  It'll look something like this:

        Callback cb = std::bind(&AStarPather::your_function_name, this);
        Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

        There are other alternatives to using std::bind, so feel free to mix it up.
        Callback is just a typedef for std::function<void(void)>, so any std::invoke'able
        object that std::function can wrap will suffice.
    */
    Callback cb = std::bind(&AStarPather::initializeMap, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    // Maximum open list size
    OpenList.reserve(40 * 40);

    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest &request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */
    if (request.settings.method != Method::ASTAR)
        return PathResult::COMPLETE;

    // If this is a new request, we clean the open list and place 
    // The start node in, also set the goal.
    if (request.newRequest)
    {
        NodeMap.Clear();
        GridPos start = terrain->get_grid_position(request.start);
        GridPos goal = terrain->get_grid_position(request.goal);
        terrain->set_color(start, Colors::Orange);
        terrain->set_color(goal, Colors::Orange);

        Node* startNode = &(NodeMap.GetNode(start.row,start.col));
        startNode->onList = ListType::OpenList;
        EndGoal = &(NodeMap.GetNode(goal.row,goal.col));

        OpenList.clear();
        // Push Start node into open list
        OpenList.emplace_back(startNode);
    }
    // Run until open list is empty.
    while (!OpenList.empty())
    {
        // Find the cheapest Node in the list
        Node*& cheapestNode = findCheapestNode();

        // If we have found the goal
        if (cheapestNode == EndGoal)
        {
            return PathResult::COMPLETE;
        }

        GetNodeInformation childNodes[8] = {};
        auto [x, y] = NodeMap.GetPosition(cheapestNode);
        NodeMap.GetAllChildNodes(x, y, childNodes);

        if (cheapestNode == nullptr)
            break;

        for (int i = 0; i < 8; ++i)
        {
            auto& child = childNodes[i];
            if (child.Possible)
            {
                // Compute g(x), from our node to child.
                // GetAllChild also computes if cost is 1 or sqrt(2
                float gx = cheapestNode->cost + child.Cost;
                float thisCost = gx + GetHeuristic(request.settings.heuristic);

                // If not in open or closed list
                if (child.childNode->onList == ListType::None)
                {
                    // Put on open list.
                    ConfigureForOpenList(child.childNode, { x,y }, thisCost, request);
                    OpenList.emplace_back(child.childNode);
                }
                else
                {
                    // If node is cheaper then the node on the open/closed list.
                    if (thisCost < child.childNode->cost)
                    {
                        // Since this is cheaper, we will use this instead.
                        // Place on open list if its not.
                        if (child.childNode->onList == ListType::ClosedList)
                        {
                            OpenList.emplace_back(child.childNode);
                        }
                        // And now we update it.
                        ConfigureForOpenList(child.childNode, { x,y }, thisCost, request);
                    }
                }
            }
        }
        // Place parent node on closed list.
        ConfigureForClosedList(cheapestNode, { x,y }, request);
        // Swap and pop idiom
        std::swap(cheapestNode, OpenList.back());
        OpenList.pop_back();

        // If taken too much time for this frame (or single step mode)
        if (request.settings.singleStep)
        {
            return PathResult::PROCESSING;
        }
    }
    // Open list empty, no possible path
    return PathResult::IMPOSSIBLE;
    
}


void AStarPather::initializeMap()
{
    std::cout << "Initializing new map" << std::endl;
    NodeMap.SetSize(terrain->get_map_width(), terrain->get_map_height());
    // Resize the node map.


    // Set all the wall nodes
    for (int i = 0; i < terrain->get_map_width(); ++i) 
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            if (terrain->is_wall(i, j))
            {
                NodeMap.GetNode(i,j).cost = WallCost;
            }
        }
    }


}

float AStarPather::GetHeuristic(Heuristic heuristic) const
{
    return 0.0f;
}

void AStarPather::ConfigureForOpenList(Node* node, GridPos gridPos, float cost, PathRequest& request)
{ 
    if (request.settings.debugColoring)
    {
        auto [x, y] = NodeMap.GetPosition(node);
        terrain->set_color(x, y, Colors::Yellow);
    }
    node->onList = ListType::OpenList;
    node->parentPosition = std::move(gridPos);
    node->cost = cost;
}

void AStarPather::ConfigureForClosedList(Node* node, GridPos gridPos, PathRequest& request)
{
    node->onList = ListType::ClosedList;
    if (request.settings.debugColoring)
    {
        terrain->set_color(gridPos, Colors::Blue);
    }
}

void AStarPather::FinalizeEndPath(PathRequest& request, Node* endNode)
{
    // Walk the end node until we reach {-1, -1}
    Node* currNode = endNode;
    // Check for -1 
    while (currNode->parentPosition.col != -1)
    {
        // If debug on
        auto worldPos = terrain->get_world_position(currNode->parentPosition);
        request.path.emplace_back(worldPos);
    }

}


Node*& AStarPather::findCheapestNode()
{
    // Greedy BSF search
    float cheapestCostSoFar = std::numeric_limits<float>::max();
    Node** cheapestNodeSoFar = nullptr;

    for (int i = 0; i < OpenList.size(); ++i)
    {
        if (cheapestCostSoFar > OpenList[i]->cost)
        {
            cheapestCostSoFar = OpenList[i]->cost;
            cheapestNodeSoFar = &OpenList[i];
        }
    }
    return *cheapestNodeSoFar;
}

void Array2D::GetAllChildNodes(int x, int y, GetNodeInformation(&arr)[8])
{
    int row_limit = terrain->get_map_width();
    int column_limit = terrain->get_map_height();
    int arrayIndex = 0;

    static const float diagonalCost = sqrtf(2.0f);
    static const float flatCost = 1.0f;
    // Go through the neighbours with bounds checking built in.
    for (int i = std::max(0, x - 1); i <= std::min(x + 1, row_limit); ++i)
    {
        for (int j = std::max(0, y - 1); j <= std::min(y + 1, column_limit); ++j)
        {
            if (x != i || y != j)
            {
                arr[arrayIndex].childNode = &this->GetNode(i,j);
                arr[arrayIndex].Cost = isStraight(x, y, i, j) ? flatCost : diagonalCost;
                arr[arrayIndex].Possible = (this->GetNode(i,j).cost != WallCost);
                ++arrayIndex;
            }
        }
    }
}

bool Array2D::isStraight(int parentX, int parentY, int childX, int childY)
{
    return (parentX == childX) || (parentY == childY);
}

void Array2D::SetSize(int width, int height)
{
    nodes.resize(width * height);
    this->width = width;
    this->height = height;
    dataPtr = nodes.data();
}

void Array2D::Reset()
{
    // nodes.clear();
}

void Array2D::Clear()
{
    memset(nodes.data(), 0, sizeof(Node) * nodes.size());
}

// Make sure you've set the size before calling this.
Node& Array2D::GetNode(int x, int y)
{
    if (x * y >= width * height)
        DebugBreak();
    if (x * y < 0)
        DebugBreak();
    return *(dataPtr + x * width + y);
}
