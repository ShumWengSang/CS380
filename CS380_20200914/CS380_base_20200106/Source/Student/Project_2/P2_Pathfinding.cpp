#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"
#include <functional>

static const float WallCost = 9999;

//#define COUT

static GridPos Min(GridPos const& a, GridPos const& b)
{
    return  {std::min(a.row, b.row), std::min(a.col, b.col)};
}

static GridPos Max(GridPos const& a, GridPos const& b)
{
    return  { std::max(a.row, b.row), std::max(a.col, b.col) };
}

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

#if defined(_DEBUG)
    if ((long)width * height <= remainder)
    {
        DebugBreak();
    }
#endif

    long long x = 0, y = 0;
    x = remainder / width;
    y = remainder - (width * x);
#if defined(_DEBUG) && defined(COUT)
    std::cout << "Returning x: " << x << " y: " << y << std::endl;
#endif
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
    //if (request.settings.method != Method::ASTAR)
    //    return PathResult::COMPLETE;

    // If this is a new request, we clean the open list and place 
    // The start node in, also set the goal.
    size_t size = sizeof(GridPosChar);
    size_t nodesize = sizeof(Node);
    if (request.newRequest)
    {
        NodeMap.Clear();
        GridPos start = terrain->get_grid_position(request.start);
        GoalPos = terrain->get_grid_position(request.goal);
        terrain->set_color(start, Colors::Orange);
        terrain->set_color(GoalPos, Colors::Orange);

        Node* startNode = &(NodeMap.GetNode(start.row,start.col));
        startNode->onList = ListType::OpenList;
        EndGoal = &(NodeMap.GetNode(GoalPos.row, GoalPos.col));

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
            FinalizeEndPath(request, cheapestNode);
#ifdef _DEBUG
            info.End();
            info.Print();
#endif
            return PathResult::COMPLETE;
        }

        GetNodeInformation childNodes[8] = {};
        auto [x, y] = NodeMap.GetPosition(cheapestNode);
        GetAllChildNodes(x, y, childNodes);

        if (cheapestNode == nullptr)
			break;

		for (int i = 0; i < 8; ++i)
		{
			auto& child = childNodes[i];
			// Negative costs are discarded, as they are deemed
			// unapproachable
			if (child.Cost >= 0)
			{
				// Compute g(x), from our node to child.
				// GetAllChild also computes if cost is 1 or sqrt(2
				auto [childX, childY] = NodeMap.GetPosition(child.childNode);
				float gx = cheapestNode->givenCost + child.Cost;
				float thisCost = gx + GetHeuristic({ childX, childY }, GoalPos, request) *
					request.settings.weight;

				// If not in open or closed list
                if (child.childNode->onList == ListType::None)
                {
                    // Put on open list.
                    ConfigureForOpenList(child.childNode, { x,y }, thisCost, gx, request);
                    OpenList.emplace_back(child.childNode);
                }
                else
                {
                    // If node is cheaper then the node on the open/closed list.
                    if (thisCost < child.childNode->finalCost)
                    {
                        // Since this is cheaper, we will use this instead.
                        // Place on open list if its not.
                        if (child.childNode->onList == ListType::ClosedList)
                        {
                            OpenList.emplace_back(child.childNode);
                        }
                        // And now we update it.
                        ConfigureForOpenList(child.childNode, { x,y }, thisCost, gx, request);
                    }
                }
            }
        }
        // Place parent node on closed list.
        ConfigureForClosedList(cheapestNode, {x, y }, request);
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
#if defined(_DEBUG) && defined(COUT)
    std::cout << "Initializing new map" << std::endl;
#endif
    NodeMap.SetSize(terrain->get_map_width(), terrain->get_map_height());
    // Resize the node map.

    // Calc split distance
    float oneDistance = DirectX::SimpleMath::Vector3::Distance(terrain->get_world_position(0, 0), terrain->get_world_position(0, 1));
    SplitDistance = oneDistance + oneDistance / 2.0f;

    // Set all the wall nodes
    for (int i = 0; i < terrain->get_map_width(); ++i) 
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            if (terrain->is_wall(i, j))
            {
                NodeMap.GetNode(i,j).finalCost = WallCost;
            }
            else
            {
                NodeMap.GetNode(i, j).finalCost = 0;
            }
        }
    }


}

float AStarPather::GetHeuristic(GridPos const &requester, GridPos const &goal, PathRequest const& heuristic) const
{
    const int xDiff = abs(requester.row - goal.row);
    const int yDiff = abs(requester.col - goal.col);
    const static float rt2 = sqrtf(2.0f);
    switch (heuristic.settings.heuristic)
    {
    case Heuristic::OCTILE:
    {
        auto min = std::min(xDiff, yDiff);
        return min * rt2 + std::max(xDiff, yDiff) - min;
        break;
    }
    case Heuristic::CHEBYSHEV:
    {
        return (float)std::max(xDiff, yDiff);
    }
    case Heuristic::MANHATTAN:
    {
        return (float)xDiff + yDiff;
    }
    case Heuristic::EUCLIDEAN:
    {
        return (float)sqrt(xDiff * xDiff + yDiff * yDiff);
    }
#ifdef _DEBUG
    default:
        DebugBreak();
#endif
    }

    return 0.0f;
}

void AStarPather::ConfigureForOpenList(Node* node, GridPos gridPos, float finalCost, float gx, PathRequest& request)
{ 
    if (request.settings.debugColoring)
    {
        auto [x, y] = NodeMap.GetPosition(node);
        terrain->set_color(x, y, Colors::Blue);
    }
    node->onList = ListType::OpenList;
    node->parentPosition = GridPosChar(gridPos); // Implicit convert
    node->finalCost = finalCost;
    node->givenCost = gx;

#ifdef _DEBUG
    info.Update(node, (int)OpenList.size());
#endif
}

void AStarPather::ConfigureForClosedList(Node* node, GridPos gridPos, PathRequest& request)
{
    node->onList = ListType::ClosedList;
    if (request.settings.debugColoring)
    {
        terrain->set_color(gridPos, Colors::Yellow);
    }
}

void AStarPather::FinalizeEndPath(PathRequest& request, Node* endNode)
{
    bool didRubberband = false;
    // If we are rubberbanding and if it works,
    if (request.settings.rubberBanding && Rubberbanding(request.path, endNode))
    {
        didRubberband = true;
    }
    else // If either are not true, we prepare the path the normal way.
    {
        NormalNodesToPath(request.path, endNode);
    }

    // If we are smoothing
    if (request.settings.smoothing)
    {
        // If we did rubberbanding, we will have to make sure the path is filled again.
        if (didRubberband)
        {
            RubberbandSmooth_AddNodes(request.path);
        }
        if (!Smoothing(request.path))
        {
            return;
        }
    }
    return ;

}


void AStarPather::RubberbandSmooth_AddNodes(WaypointList& path)
{
    if (path.size() < 2)
        return;

    WaypointList::iterator first = path.begin();
    WaypointList::iterator second= std::next(first);

    while (second != path.end())
    {
        Split(first, second, path);
        first = second;
        ++second;
    }
}

bool AStarPather::Split(WaypointList::iterator const& a, WaypointList::iterator const& b, WaypointList& path)
{
    // Base case
    if (DirectX::SimpleMath::Vector3::Distance(*a, *b) <= SplitDistance)
        return false;
    else
    {
        // Create a node in between the two points.
        Vec3 midPoint = (*a + *b) / 2;
        WaypointList::iterator newIterator = path.emplace(b, midPoint);
        // Now split between the new point and a, new point and b
        return Split(a, newIterator, path) || Split(newIterator, b, path);
    }
}

bool AStarPather::Smoothing(WaypointList& path)
{
    if (path.empty())
        return false;

    // We add one before and one at the end of the path.
    path.emplace_front(path.front());
    path.emplace_back(path.back());

    // Make sure there are enough points to work on.
    WaypointList::iterator left = path.begin();
    if (left == path.end())
        return false;
    WaypointList::iterator midLeft = std::next(left);
    if (midLeft == path.end())
        return false;
    WaypointList::iterator midRight = std::next(midLeft);
    if (midRight == path.end())
        return false;
    WaypointList::iterator right = std::next(midRight);
    if (right == path.end())
        return false;

    while (right != path.end())
    {
        using DirectX::SimpleMath::Vector3;
        path.emplace(midRight, Vector3::CatmullRom(*left, *midLeft, *midRight, *right, 0.25f));
        path.emplace(midRight, Vector3::CatmullRom(*left, *midLeft, *midRight, *right, 0.50f));
        path.emplace(midRight, Vector3::CatmullRom(*left, *midLeft, *midRight, *right, 0.75f));
        
        // Advance Iterators
        left = midLeft;
        midLeft = midRight;
        ++midRight, ++right;
    }

    return true;
}

bool AStarPather::Rubberbanding(WaypointList& path, Node* endNode)
{
    // For 3 nodes, make a square and determine if any of the squares are walls. 
    // If so, don't remove the middle node. Else remove middle node.


    // Get all three ptrs.
    Node* tailNode = endNode;
    if (!tailNode)
        return false;
    Node* middleNode = GetNextNode(tailNode);
    if (!middleNode)
        return false;
    Node* headNode = GetNextNode(middleNode);
    if (!headNode)
        return false;

#if defined(_DEBUG) && defined(COUT)
    // DEBUG PRINT ALL NODES
    Node* curNode = tailNode;
    while (curNode != nullptr)
    {
        std::cout << "Vec2 [" << curNode->parentPosition.row << "," << curNode->parentPosition.col << "]" << std::endl;
        curNode = GetNextNode(curNode);
    }
#endif

    PlaceThisIntoPath(path, tailNode);

    // Keep doing this until head is null.
    while (headNode != nullptr)
    {
        GridPos headPos = GetPosition(headNode);
        GridPos tailPos = GetPosition(tailNode);
        GridPos midPos = GetPosition(middleNode);
        // Create a square from tail and head
        //GridPos RectSize = { abs(tailPos.row - headPos.row), abs(tailPos.col - headPos.col) };
        GridPos startPoint = Min(headPos, tailPos);
        GridPos endPoint = Max(headPos, tailPos);
        
#if defined(_DEBUG) && defined(COUT)
        std::cout << "Head position: " << headPos.row << "," << headPos.col << 
            " and tail position: " << tailPos.row << "," << tailPos.col << std::endl;
#endif

        // Go through each node in that square
        if (isWallInRange(startPoint.row, startPoint.col, endPoint.row, endPoint.col))
        {
#if defined(_DEBUG) && defined(COUT)
            std::cout << "Wall detected" << std::endl;
#endif

            // There is a wall, we need the middle node. 
            PlaceThisIntoPath(path, middleNode);
            headNode = GetNextNode(headNode);
            tailNode = middleNode;
            middleNode = GetNextNode(middleNode);

        }
        else
        {
            // No walls, we don't add it in.
            headNode = GetNextNode(headNode);
            middleNode = GetNextNode(middleNode);
        }
    }

    // At the end of it, we need to add the last node in.
    PlaceThisIntoPath(path, &NodeMap.GetNode(middleNode->parentPosition));
    return true;
}

void AStarPather::NormalNodesToPath(WaypointList& path, Node* endNode)
{
    // Error check
    if (endNode->parentPosition.col == -1)
        return;
    Node* currNode = endNode;
    PlaceThisIntoPath(path, currNode);
    while (currNode != nullptr)
    {
        PlaceParentIntoPath(path, currNode);
        currNode = GetNextNode(currNode);
    }
}


Node* AStarPather::GetNextNode(Node* node)
{
    Node* currNode = &NodeMap.GetNode(node->parentPosition);
    return currNode->parentPosition.col == -1 ? nullptr : currNode;
}

void AStarPather::PlaceParentIntoPath(WaypointList& path, Node* ptr)
{
    auto worldPos = terrain->get_world_position({ ptr->parentPosition.row, ptr->parentPosition.col });
    path.emplace_front(worldPos);
}

void AStarPather::PlaceThisIntoPath(WaypointList& path, Node* ptr)
{
    auto [x, y] = NodeMap.GetPosition(ptr);
    auto worldPos = terrain->get_world_position(x, y);
    path.emplace_front(worldPos);
}

GridPos AStarPather::GetPosition(Node* node)
{
    auto [x, y] = NodeMap.GetPosition(node);
    return { x,y };
}

bool AStarPather::isWallInRange(int xBegin, int yBegin, int xEnd, int yEnd)
{
    for (auto i = xBegin; i <= xEnd; ++i)
    {
        for (auto j = yBegin; j <= yEnd; ++j)
        {

            if (terrain->is_wall(i, j))
            {
                return true;
            }
        }
    }
    return false;
}


float AStarPather::CalcCost(int parentX, int parentY, int childX, int childY)
{
    static const float diagonalCost = sqrtf(2.0f);
    static const float flatCost = 1.0f;
    static const float FailCost = -1;

    // If this is a wall, just fail
    if ((NodeMap.GetNode(childX, childY).finalCost == WallCost))
        return FailCost;

    //If straight, cost is 1.
    if (NodeMap.isStraight(parentX, parentY, childX, childY))
        return flatCost;

    // Else now it is diagonal
    else
    {
        // Corresponding checks to dx,dy for diagonality

        // Now do diagonal checks
        int dx = parentX - childX;
        int dy = parentY - childY;

        // Two wall check, if either are wall we fail
        if (NodeMap.GetNode(childX, childY + dy).finalCost == WallCost ||
            NodeMap.GetNode(childX + dx, childY).finalCost == WallCost)
        {
            return FailCost;
        }
        else
            return diagonalCost;
    }

}


Node*& AStarPather::findCheapestNode()
{
    // Greedy BSF search
    float cheapestCostSoFar = std::numeric_limits<float>::max();
    Node** cheapestNodeSoFar = nullptr;

    std::for_each(OpenList.rbegin(), OpenList.rend(), [&](Node*& node)
        {
            if (cheapestCostSoFar > node->finalCost)
            {
                cheapestCostSoFar = node->finalCost;
                cheapestNodeSoFar = &node;
            }
        });

    return *cheapestNodeSoFar;
}

void AStarPather::GetAllChildNodes(int x, int y, GetNodeInformation(&arr)[8])
{
    int row_limit = terrain->get_map_width();
    int column_limit = terrain->get_map_height();
    int arrayIndex = 0;

    // Go through the neighbours with bounds checking built in.
    for (int i = std::max(0, x - 1); i <= std::min(x + 1, row_limit - 1); ++i)
    {
        for (int j = std::max(0, y - 1); j <= std::min(y + 1, column_limit - 1); ++j)
        {
            if (x != i || y != j)
            {
                arr[arrayIndex].childNode = &NodeMap.GetNode(i,j);
                arr[arrayIndex].Cost = CalcCost(x, y, i, j);
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
    for (int i = 0; i < nodes.size(); ++i)
    {
        // Ignore if wall
        if(nodes[i].finalCost != WallCost)
            nodes[i] = Node();
    }
}

// Make sure you've set the size before calling this.
Node& Array2D::GetNode(int x, int y)
{
#if defined(_DEBUG)
    if (x * y >= width * height)
        DebugBreak();
    if (x < 0)
        DebugBreak();
    if (y < 0)
        DebugBreak();
    if (x * y < 0)
        DebugBreak();
    if (x >= width)
        DebugBreak();
    if (y >= height)
        DebugBreak();
#endif
    auto index = (dataPtr + x * width + y) - dataPtr;
#if defined(_DEBUG) && defined(COUT)
    std::cout << "Request of " << x << " : " << y << " gives " << index << " thing." << std::endl;
#endif
    return *(dataPtr + x * width + y);
}

Node& Array2D::GetNode(GridPosChar position)
{
    return GetNode(position.row, position.col);
}
