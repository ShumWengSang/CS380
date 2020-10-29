#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

// Child node should be the [x1, y1]
static bool isWallBlockingDiag(int x0, int y0, int x1, int y1)
{
    int dx = x0 - x1;
    int dy = y0 - y1;

    return terrain->is_wall(x1, y1 + dy) || terrain->is_wall(x1 + dx, y1);
}

static bool IsLayerEmpty(MapLayer<float>& layer)
{
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            float value = layer.get_value(i0, j0);
            if (value != 0)
            {
                return false;
            }
        }
    }
    return true;
}

static bool NeighbourVisibiilityCheck(int i0, const int& row_limit, int j0, const int& column_limit, int row, int col)
{
    // Check against the neighbors with gurantees against the map boundaries
    for (int i1 = std::max(0, i0 - 1); i1 <= std::min(i0 + 1, row_limit - 1); ++i1)
    {
        for (int j1 = std::max(0, j0 - 1); j1 <= std::min(j0 + 1, column_limit - 1); ++j1)
        {
            if (i0 != i1 || j0 != j1)
            {
                // If any of the neighbors can see the given point
                if (is_clear_path(i1, j1, row, col))
                {
                    // Do the wall check
                    if (!isWallBlockingDiag(i0, j0, i1, j1))
                    {
                        // 0.5f
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */
    // Store the potential closest distance walls/empty spaces
    std::vector<std::pair<int, int>> potentialWalls;
    potentialWalls.reserve(40 * 40); 
    float bestDistanceSquared = std::numeric_limits<float>::max();

    // Get the distance between tiles in a map.
    const Vec3 dist = terrain->get_world_position(0, 0) - terrain->get_world_position(1, 1);

    // For every map
    for (int i = -1; i <= terrain->get_map_width(); ++i)
    {
        for (int j = -1; j <= terrain->get_map_height(); ++j)
        {
            if (row == i && col == j)
                continue;
            // If position is invalid
            if (!terrain->is_valid_grid_position(i, j) || terrain->is_wall(i,j))
            {
                potentialWalls.emplace_back(std::make_pair(i, j));
            }
            // If position is a wall
            else if (terrain->is_wall(i, j))
            {
                potentialWalls.emplace_back(std::make_pair(i, j));
            }
        }
    }

    // Now we do BSF to find closest distance
    for (auto& gridPos : potentialWalls)
    {
        const float xDiff = float(gridPos.first - row);
        const float yDiff = float(gridPos.second - col);
        float currentDistanceSquared = xDiff * xDiff + yDiff * yDiff;
        if (bestDistanceSquared > currentDistanceSquared)
        {
            bestDistanceSquared = currentDistanceSquared;
        }
    }
    
    return bestDistanceSquared;
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // We only need to engage the square between gridPos A and gridPos B
    int smallerRol, smallerCol;
    int largerRow, largerCol;
    
    // Find the smaller row
    if (row0 < row1)
    {
        smallerRol = row0;
        largerRow = row1;
    }
    else
    {
        smallerRol = row1;
        largerRow = row0;
    }

    // Find the smaller col
    if (col0 < col1)
    {
        smallerCol = col0;
        largerCol = col1;
    }
    else
    {
        smallerCol = col1;
        largerCol = col0;
    }

    // Store walls here.
    std::vector<std::pair<int, int>> Walls;
    Walls.reserve(40 * 40);

    // Go through the square and make check for walls
    for (int i = smallerRol; i <= largerRow; ++i)
    {
        for (int j = smallerCol; j <= largerCol; ++j)
        {
            if(terrain->is_wall(i,j))
                Walls.emplace_back(std::make_pair(i, j));
        }
    }
    // Lets get the difference in one tile in world position. Assuming grids are square
    // Vec2.x == Vec2.y
    const float worldDistance = std::abs(terrain->get_world_position(0, 0).x - terrain->get_world_position(1, 1).x);
    const float halfDistance = worldDistance / 2.0f;
    const float puffValue = 0.01f;
    const float moveValue = halfDistance + puffValue * worldDistance;
    
    // Prepare the line between the two grids.
    const Vec3& v3_point0 = terrain->get_world_position(row0, col0);
    const Vec3& v3_point1 = terrain->get_world_position(row1, col1);
    const Vec2 point0 = { v3_point0.x, v3_point0.z };
    const Vec2 point1 = { v3_point1.x, v3_point1.z };

    // Go through each wall and check if intersect
    for (auto& wall : Walls)
    {
        // Get the world position to form the 4 lines that form the wall.
        const auto& worldPos = terrain->get_world_position(wall.first, wall.second);
        // Get the four points that form the grid, we readadd the first point
        // so the loop is easier later
        const Vec2 points[] = {
            {worldPos.x + moveValue, worldPos.z + moveValue},
            {worldPos.x - moveValue, worldPos.z + moveValue},
            {worldPos.x - moveValue, worldPos.z - moveValue},
            {worldPos.x + moveValue, worldPos.z - moveValue},
            {worldPos.x + moveValue, worldPos.z + moveValue}
        };
        
        // Now we check against the 4 lines that form the wall
        for (int i = 1; i < sizeof(points) / sizeof(points[0]); ++i)
        {
            if (line_intersect(point0, point1, points[i - 1], points[i]))
            {
                // We have an intersection. Not clear sight.
                return false;
            }
        }
    }

    return true; 
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */
    for (int i = 0; i < terrain->get_map_width(); ++i)
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            if (!terrain->is_wall(i, j))
            {
                float closestDistance = distance_to_closest_wall(i, j);
                layer.set_value(i, j, 1 / closestDistance);
            }
        }
    }
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */
    // For each grid.
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            // Check against other grid.
            int numVisibleCells = 0;
            
            // If this is a wall, just skip
            if (terrain->is_wall(i0, j0))
                continue;

            // We check against every other cell in the grid.
            for (int i1 = 0; i1 < terrain->get_map_width(); ++i1)
            {
                for (int j1 = 0; j1 < terrain->get_map_height(); ++j1)
                {
                    if (is_clear_path(i0, j0, i1, j1))
                    {
                        ++numVisibleCells;
                    }
                }
            }
            layer.set_value(i0, j0, std::min(numVisibleCells / 160.0f, 1.0f));
        }
    }
}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // For each grid.
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            // If this is a wall, just skip
            if (terrain->is_wall(i0, j0))
                continue;

            // Am I visible to the given position?
            if (is_clear_path(i0, j0, row, col))
            {
                // Yes, so mark with a 1.0
                layer.set_value(i0, j0, 1.0f);
            }
            else 
            {
                const int row_limit = terrain->get_map_width();
                const int column_limit = terrain->get_map_height();

                // If any neighbors see it, 0.5f
                if (NeighbourVisibiilityCheck(i0, row_limit, j0, column_limit, row, col))
                {
                    layer.set_value(i0, j0, 0.5f);
                }
                else
                {
                    // Nothing sees it. 0
                    layer.set_value(i0, j0, 0.0f);
                }
            }
        }
    }
}


void analyze_agent_vision(MapLayer<float>& layer, const Agent* agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */
    // Dot product of view and from agent to cell
    auto AgentView_3d = agent->get_forward_vector();
    Vec2 agentView = { AgentView_3d.x, AgentView_3d.z };
    agentView.Normalize();

    // For each grid.
    for (int i = 0; i < terrain->get_map_width(); ++i)
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            // If this is a wall, just skip
            if (terrain->is_wall(i, j))
                continue;

            // Vector from agent to cell
            Vec3 agentToCell = { terrain->get_world_position(i, j) - agent->get_position() };
            Vec2 agentToCellNorm = { agentToCell.x, agentToCell.z };
            agentToCell.Normalize();

            // Dot Product between the two
            float dotProuct = agentToCellNorm.Dot(agentView);

            // Check against the cos
            const float FoVDeg = 185;
            const float FoVRad = FoVDeg * PI / 180.f;

            // Now we see if the current node enters the FOV of agent
            if (dotProuct > cos(FoVRad))
            {
                // Now we check if we can see node
                const auto& agentPos = terrain->get_grid_position(agent->get_position());
                if (is_clear_path(i, j, agentPos.row, agentPos.col))
                {
                    layer.set_value(i, j, 1.0f);
                }
            }
        }
    }
}

float Distance(int i0, int j0, int i1, int j1)
{
    return Vec2::Distance({ (float)i0,(float)j0 }, { (float)i1,(float)j1 });
}

float DecayFormula(float old_influence, float distance, float decayFactor)
{
    return old_influence * exp(-1 * distance * decayFactor);
}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    // If everything is zero, then we leave the map alone
    if (IsLayerEmpty(layer))
    {
        return;
    }

    float tempLayer[40][40] = { 0 };
    const unsigned tempLayerSize = sizeof(tempLayer) / sizeof(float);

    // For every cell
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            // Continue loop if its a wall
            if (terrain->is_wall(i0, j0))
            {
                continue;
            }

            // Record the maximum decayed influence
            float maximumDecay = 0;
            // Get the value of each neighbor and apply decay factor
            for (int i1 = std::max(0, i0 - 1); i1 <= std::min(i0 + 1, terrain->get_map_width() - 1); ++i1)
            {
                for (int j1 = std::max(0, j0 - 1); j1 <= std::min(j0 + 1, terrain->get_map_height() - 1); ++j1)
                {
                    // Exclude the current node
                    if (i0 != i1 || j0 != j1)
                    {
                        // Make sure its not between a wall
                        if (!isWallBlockingDiag(i0, j0, i1, j1))
                        {
                            // Apply decay factor to neighbour node
                            float nodeDecay = DecayFormula(layer.get_value(i1, j1), Distance(i0, j0, i1, j1), decay);

                            // Record maxdecay
                            if (nodeDecay > maximumDecay)
                            {
                                maximumDecay = nodeDecay;
                            }
                        }
                    }
                }
            }

            // Apply linear interpolation 
            float res = lerp(layer.get_value(i0, j0), maximumDecay, growth);

            // Store in temp layer
            tempLayer[i0][j0] = res;
        }
    }
    
    // Copy back to layer. We can't use memcpy because map width/height aren't guranteed
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            layer.set_value(i0, j0, tempLayer[i0][j0]);
        }
    }
}


void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    float tempLayer[40][40] = { 0 };
    const unsigned tempLayerSize = sizeof(tempLayer) / sizeof(float);

    if (IsLayerEmpty(layer))
    {
        return;
    }

    // For every cell
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            // Continue loop if its a wall
            if (terrain->is_wall(i0, j0))
            {
                continue;
            }

            // Record the maximum decayed influence
            float maximumDecay = std::numeric_limits<float>::min();
            GridPos maximumDecayPos = { -1,-1 };
            // Get the value of each neighbor and apply decay factor
            for (int i1 = std::max(0, i0 - 1); i1 <= std::min(i0 + 1, terrain->get_map_width() - 1); ++i1)
            {
                for (int j1 = std::max(0, j0 - 1); j1 <= std::min(j0 + 1, terrain->get_map_height() - 1); ++j1)
                {
                    // Exclude the current node or if its a wall
                    if (i0 != i1 || j0 != j1)
                    {
                        // Make sure its not a wall
                        if (!isWallBlockingDiag(i0, j0, i1, j1))
                        {
                            // Apply decay factor to neighbour node
                            float nodeDecay = DecayFormula(layer.get_value(i1, j1), Distance(i0, j0, i1, j1), decay);

                            // Record maxdecay
                            if (std::abs(nodeDecay) > std::abs(maximumDecay))
                            {
                                maximumDecay = nodeDecay;
                                maximumDecayPos = GridPos({ i1, j1 });
                            }
                        }
                    }
                }
            }

            // Apply linear interpolation 
            float res = lerp(layer.get_value(i0, j0), maximumDecay, growth);

            // Store in temp layer
            tempLayer[i0][j0] = res;
        }
    }

    // Copy back to layer. We can't use memcpy because map width/height aren't guranteed
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            layer.set_value(i0, j0, tempLayer[i0][j0]);
        }
    }
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    // Find max value
    float maximumValue = 0;
    layer.for_each([&maximumValue](float& value) -> void
        {
            if (value > maximumValue)
            {
                maximumValue = value;
            }
        });

    if (maximumValue == 0)
        return;

    // Normalize all the values.
    layer.for_each([&maximumValue](float& value) -> void
        {
            value = value / maximumValue;
        });
}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */
    // Find max value
    if (IsLayerEmpty(layer))
    {
        return;
    }

    float maximumValue = 0;
    float lowestNegative = 0;
    layer.for_each([&maximumValue, & lowestNegative](float& value) -> void
        {
            // If Positive value
            if (value > 0)
            {
                if (value > maximumValue)
                {
                    maximumValue = value;
                }
            }
            // Else negative value
            else if (value < 0)
            {
                if (value < lowestNegative)
                {
                    lowestNegative = value;
                }
            }
        });

    if (maximumValue == 0)
        return;

    // Normalize all the values.
    layer.for_each([&maximumValue, &lowestNegative](float& value) -> void
        {
            // If positive
            if (value > 0)
            {
                value = value / maximumValue;
            }
            else if (value < 0)
            {
                value = value / lowestNegative;
                value *= -1;
            }
        });
    
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    // Set any negative values to 0
    for (int i0 = 0; i0 < terrain->get_map_width(); ++i0)
    {
        for (int j0 = 0; j0 < terrain->get_map_height(); ++j0)
        {
            // If value is negative
            if (layer.get_value(i0, j0) < 0)
            {
                // Set to 0
                layer.set_value(i0, j0, 0);
            }
        }
    }


    // Dot product of view and from agent to cell
    auto AgentView_3d = enemy->get_forward_vector();
    Vec2 agentView = { AgentView_3d.x, AgentView_3d.z };
    agentView.Normalize();

    // For each grid.
    for (int i = 0; i < terrain->get_map_width(); ++i)
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            // If this is a wall, just skip
            if (terrain->is_wall(i, j))
                continue;

            // If tile is close enough then we just use a radius check
            if (Vec3::Distance(enemy->get_position(), terrain->get_world_position(i, j)) < closeDistance)
            {
                auto enemyGridPos = terrain->get_grid_position(enemy->get_position());
                // If there is a clear path to the node
                if (is_clear_path(i, j, enemyGridPos.row, enemyGridPos.col))
                {
                    layer.set_value(i, j, occupancyValue);
                }
            }
            else // Else use FOV 
            {
                // Vector from agent to cell
                Vec3 agentToCell = { terrain->get_world_position(i, j) - enemy->get_position() };
                Vec2 agentToCellNorm = { agentToCell.x, agentToCell.z };
                agentToCell.Normalize();

                // Dot Product between the two
                float dotProuct = agentToCellNorm.Dot(agentView);

                // Check against the cos
                const float FoVDeg = 185;
                const float FoVRad = FoVDeg * PI / 180.f;

                // Now we see if the current node enters the FOV of agent
                if (dotProuct > cos(FoVRad))
                {
                    // Now we check if we can see node
                    const auto& agentPos = terrain->get_grid_position(enemy->get_position());
                    if (is_clear_path(i, j, agentPos.row, agentPos.col))
                    {
                        layer.set_value(i, j, occupancyValue);
                    }
                }
            }
        }
    }
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // Find max non-zero value
    std::vector<GridPos> arrOfGridPos;
    arrOfGridPos.reserve(40 * 40);
    float maximumValue = 0;

    GridPos agentPos = terrain->get_grid_position(enemy->get_position());

    for (int i = 0; i < terrain->get_map_width(); ++i)
    {
        for (int j = 0; j < terrain->get_map_height(); ++j)
        {
            // Ignore the agent position
            if (i == agentPos.row && j == agentPos.col)
                continue;
            float value = std::abs(layer.get_value(i, j));
            if (value == maximumValue)
            {
                arrOfGridPos.emplace_back(GridPos{ i,j });
            }
            // New max discovered
            if (value > maximumValue)
            {
                // Clear array
                arrOfGridPos.clear();
                arrOfGridPos.emplace_back(GridPos{ i,j });
                maximumValue = value;
            }
        }
    }


    // Now we find the closest to the player
    if (arrOfGridPos.size() > 0)
    {
        // Find the closest to player
        float curMinDistance = std::numeric_limits<float>::max();
        Vec3 currMinDistPos;
        for each (const auto& gridPosition in arrOfGridPos)
        {
            Vec3 worldPosNode = terrain->get_world_position(gridPosition);
            // Find the distance between curr node to enemy.
            float dist = Vec3::Distance(enemy->get_position(), worldPosNode);
            if (dist < curMinDistance)
            {
                curMinDistance = dist;
                currMinDistPos = worldPosNode;
            }
        }
        // Now we make our enemy go there
        enemy->path_to(currMinDistPos);
        return true;
    }
    else
    {
        return false;
    }
}
