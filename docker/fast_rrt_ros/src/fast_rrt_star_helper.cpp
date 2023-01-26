#include "fast_rrt_star_helper.h"

#include <ros/ros.h>
#include <cmath>

using namespace std;

namespace fast_rrt_star_planner
{
  world_coords Fast_RRTStarHelper::distance(world_coords x0, world_coords y0, world_coords x1, world_coords y1)
  {
    // World (x,y) coordinate difference between candidate and random nodes
    
    /**
     * World (x,y) coordinate difference between candidate and random nodes
     */
    world_coords diff_x = x0 - x1;
    world_coords diff_y = y0 - y1;

    /**
     * Compute Euclidean distance between points
     */
    world_coords dist = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

    return dist;
  }

  world_coords Fast_RRTStarHelper::distance(Fast_RRTStarNode *node0, Fast_RRTStarNode *node1)
  {
    return distance(node0->x, node0->y, node1->x, node1->y);
  }

  double Fast_RRTStarHelper::angle(Fast_RRTStarNode *node0, Fast_RRTStarNode *node1)
  {
    world_coords wdx = node1->x - node0->x;
    world_coords wdy = node1->y - node0->y;

    double angle = atan2(wdy, wdx);

    return angle;
  }

  bool Fast_RRTStarHelper::isLineCollisionFree(costmap_2d::Costmap2D *costmap, map_cost lethal_cost, world_coords wx0, world_coords wy0, world_coords wx1, world_coords wy1)
  {
    /**
     * Bresenham Ray-Tracing
     */
    vector<base_local_planner::Position2DInt> ray_traced_line = Bresenham(costmap, wx0, wy0, wx1, wy1);

    for (size_t i = 0; i < ray_traced_line.size(); ++i)
    {
      /**
       * Get cost
       */
      map_coords mx = ray_traced_line[i].x;
      map_coords my = ray_traced_line[i].y;

      if (isPointCollisionFree(costmap, lethal_cost, mx, my) == false)
      {
        return false;
      }
    }

    /**
     * If no points had collisions, then the whole
     * line has no collisions
     */
    return true;
  }

  bool Fast_RRTStarHelper::isPointCollisionFree(costmap_2d::Costmap2D *costmap, map_cost lethal_cost, map_coords mx, map_coords my)
  {
    map_cost cell_cost = costmap->getCost(mx, my);

    if (cell_cost >= lethal_cost)
    {
      return false;
    }

    return true;
  }

  vector<base_local_planner::Position2DInt> Fast_RRTStarHelper::Bresenham(costmap_2d::Costmap2D *costmap, Fast_RRTStarNode *start_node, Fast_RRTStarNode *end_node)
  {
    if (start_node == nullptr)
    {
      ROS_ERROR("NULLPTR: start_node");
    }

    if (end_node == nullptr)
    {
      ROS_ERROR("NULLPTR: end_node");
    }

    world_coords start_wx = start_node->x;
    world_coords start_wy = start_node->y;
    world_coords end_wx = end_node->x;
    world_coords end_wy = end_node->y;

    return Bresenham(costmap, start_wx, start_wy, end_wx, end_wy);
  }

  vector<base_local_planner::Position2DInt> Fast_RRTStarHelper::Bresenham(costmap_2d::Costmap2D *costmap, world_coords wx0, world_coords wy0, world_coords wx1, world_coords wy1)
  {
    map_coords mx0, mx1, my0, my1;

    costmap->worldToMap(wx0, wy0, mx0, my0);
    costmap->worldToMap(wx1, wy1, mx1, my1);

    return Bresenham(mx0, my0, mx1, my1);
  }

  vector<base_local_planner::Position2DInt> Fast_RRTStarHelper::Bresenham(map_coords x0, map_coords y0, map_coords x1, map_coords y1)
  {
    vector<base_local_planner::Position2DInt> ray_traced_line;

    base_local_planner::FootprintHelper fh;
    fh.getLineCells(x0, x1, y0, y1, ray_traced_line);

    return ray_traced_line;
  }

  bool Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_2d::Costmap2D *costmap, costmap_2d::Costmap2DROS *costmap_ros, map_cost lethal_cost, Fast_RRTStarNode *node0, Fast_RRTStarNode *node1)
  {
    /**
     * Bresenham Ray-Tracing
     */
    vector<base_local_planner::Position2DInt> ray_traced_line = Bresenham(costmap, node0, node1);

    for (size_t i = 0; i < ray_traced_line.size(); ++i)
    {
      /**
       * Check center point of robot
       */
      map_coords mx = ray_traced_line[i].x;
      map_coords my = ray_traced_line[i].y;
      world_coords wx, wy;
      costmap->mapToWorld(mx, my, wx, wy);

      if (isPointCollisionFree(costmap, lethal_cost, mx, my) == false)
      {
        return false;
      }

      /**
       * Get footprint and angle
       */
      vector<geometry_msgs::Point> base_footprint = costmap_ros->getRobotFootprint();
      double theta = angle(node0, node1);

      /**
       * Need to transform footprint points to a theoretical
       * to check for collisions
       *
       * Construct rotation matrix for yaw angle theta
       * R_z(theta) = [ cos(theta), -sin(theta), 0;
       *                sin(theta), cos(theta),  0;
       *                0,          0,           1 ];
       */
      vector<double> R_z_theta = {cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0};

      /**
       * Construct translation matrix
       * t(tx, ty, tz) = [ tx;
       *                   ty;
       *                   tz;
       *                   1; ]
       */
      vector<double> t = {wx, wy, 0};

      /**
       * Transform base_footprint points with transformation matrix
       * and populate oriented_footprint vector
       */
      vector<geometry_msgs::Point> oriented_footprint;
      for (size_t i = 0; i < base_footprint.size(); ++i)
      {
        geometry_msgs::Point old_pt = base_footprint[i];
        geometry_msgs::Point new_pt;

        /**
         * Do matrix multiplication manually
         */
        new_pt.x = old_pt.x * R_z_theta[0] + old_pt.y * R_z_theta[1] + old_pt.z * R_z_theta[2] + t[0];
        new_pt.y = old_pt.x * R_z_theta[3] + old_pt.y * R_z_theta[4] + old_pt.z * R_z_theta[5] + t[1];
        new_pt.z = old_pt.x * R_z_theta[6] + old_pt.y * R_z_theta[7] + old_pt.z * R_z_theta[8] + t[2];

        oriented_footprint.push_back(new_pt);
      }

      /**
       * Check lines connecting footprint vertices for collisions
       */
      for (size_t i = 0; i < oriented_footprint.size(); ++i)
      {
        world_coords wx0 = oriented_footprint[i].x;
        world_coords wy0 = oriented_footprint[i].y;
        world_coords wx1, wy1;

        if (i + 1 < oriented_footprint.size())
        {
          wx1 = oriented_footprint[i + 1].x;
          wy1 = oriented_footprint[i + 1].y;
        }
        else
        {
          wx1 = oriented_footprint[0].x;
          wy1 = oriented_footprint[0].y;
        }

        if (isLineCollisionFree(costmap, lethal_cost, wx0, wy0, wx1, wy1) == false)
        {
          return false;
        }
      }
    }

    return true;
  }
}