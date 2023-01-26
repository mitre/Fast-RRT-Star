#ifndef FAST_RRT_STAR_HELPER_H
#define FAST_RRT_STAR_HELPER_H

#include "fast_rrt_star_node.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/footprint_helper.h>
#include <vector>

/**
 * Create type aliases for map coordinates and cost
 */
using map_coords = unsigned int;
using map_cost = unsigned char;

namespace fast_rrt_star_planner
{
  /**
   * @brief Class with helper functions for Fast_RRTStarPlanner and Fast_RRTStarNode classes
   */
  class Fast_RRTStarHelper
  {
  public:
    /**
     * @brief Find euclidean distance between two world coordinate points 
     * 
     * @param x0 First position x axis coordinate
     * @param y0 First position y axis coordinate
     * @param x1 Second position x axis coordinate
     * @param y1 Second position y axis coordinate
     * @return world_coords Euclidean distance between (x0,y0) and (x1,y1)
     */
    static world_coords distance(world_coords x0, world_coords y0, world_coords x1, world_coords y1);

    /**
     * @brief Find euclidean distance between two nodes
     *
     * @param node0 First node
     * @param node1 Second node
     * @return world_coords Euclidean distance between node0 and node1
     */
    static world_coords distance(Fast_RRTStarNode *node0, Fast_RRTStarNode *node1);

    /**
     * @brief Find angle of line from node0 to node1
     *        relative to 0 radians being to the right
     *        (i.e., just like a unit circle in XY plane)
     *
     * @param node0 First node
     * @param node1 Second node
     * @return double Angle of line from node0 to node1 in radians
     */
    static double angle(Fast_RRTStarNode *node0, Fast_RRTStarNode *node1);

    /**
     * @brief Checks if line between two world coordinates is collision free 
     *
     * @param costmap Map of environment
     * @param lethal_cost Cost map value that corresponds to a region that is inaccessible
     * @param wx0 X component of first point in world coordinates
     * @param wy0 Y component of first point in world coordinates
     * @param wx1 X component of second point in world coordinates
     * @param wy1 Y coponent of second point in world coordinates
     * @return true If line from (wx0,wy0) to (wx1,wy1) doesn't have collisions
     * @return false If line from (wx0,wy0) to (wx1,wy1) does have collisions
     */
    static bool isLineCollisionFree(costmap_2d::Costmap2D *costmap, map_cost lethal_cost, world_coords wx0, world_coords wy0, world_coords wx1, world_coords wy1);

    /**
     * @brief Checks if map coordinate point is collision free
     * 
     * @param costmap Map of environment
     * @param lethal_cost Cost map value that corresponds to a region that is inaccessible
     * @param mx X component of point in map coordinates
     * @param my Y component of point in map coordinates
     * @return true If point has not collisions
     * @return false If point does have collisions
     */
    static bool isPointCollisionFree(costmap_2d::Costmap2D *costmap, map_cost lethal_cost, map_coords mx, map_coords my);

    /**
     * @brief Discretizes continuous line into grid cells and returns a vector of base_local_planner::Position2DInt.
     * 
     * @param costmap Map of environment
     * @param start_node Starting node
     * @param end_node Ending node
     * @return std::vector<base_local_planner::Position2DInt> Vector of costmap cells that ray traced line passes through
     */
    static std::vector<base_local_planner::Position2DInt> Bresenham(costmap_2d::Costmap2D *costmap, Fast_RRTStarNode *start_node, Fast_RRTStarNode *end_node);
    
    /**
     * @brief Discretizes continuous line into grid cells and returns a vector of base_local_planner::Position2DInt.
     * 
     * @param costmap Map of environment
     * @param wx0 X component of first point in world coordinates
     * @param wy0 Y component of first point in world coordinates
     * @param wx1 X component of second point in world coordinates
     * @param wy1 Y component of second point in world coordinates
     * @return std::vector<base_local_planner::Position2DInt> Vector of costmap cells that ray traced line passes through
     */
    static std::vector<base_local_planner::Position2DInt> Bresenham(costmap_2d::Costmap2D *costmap, world_coords wx0, world_coords wy0, world_coords wx1, world_coords wy1);
    
    /**
     * @brief Discretizes continuous line into grid cells and returns a vector of base_local_planner::Position2DInt.
     * 
     * @param x0 X component of first point in map coordinates
     * @param y0 Y component of first point in map coordinates
     * @param x1 X component of second point in map coordinates
     * @param y1 Y component of second point in map coordinates
     * @return std::vector<base_local_planner::Position2DInt> Vector of costmap cells that ray traced line passes through
     */
    static std::vector<base_local_planner::Position2DInt> Bresenham(map_coords x0, map_coords y0, map_coords x1, map_coords y1);

    /**
     * @brief Checks if path between two nodes is collision free when accounting for robot footprint and orientation 
     *
     * @param costmap Map of environment
     * @param costmap_ros ROS version of map of environment
     * @param lethal_cost Cost map value that corresponds to a region that is inaccessible
     * @param node0 First node
     * @param node1 Second node
     * @return true If path has no collisions with footprint of robot
     * @return false If path does have collisions with footprint of robot
     */
    static bool isFootprintPathCollisionFree(costmap_2d::Costmap2D *costmap, costmap_2d::Costmap2DROS *costmap_ros, map_cost lethal_cost, Fast_RRTStarNode *node0, Fast_RRTStarNode *node1);
  };
}

#endif /* FAST_RRT_STAR_HELPER_H */