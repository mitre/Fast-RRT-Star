#include "fast_rrt_star_node.h"
#include "fast_rrt_star_helper.h"

using namespace std;

namespace fast_rrt_star_planner
{
  Fast_RRTStarNode::Fast_RRTStarNode()
    : x(0.0), y(0.0), parent(nullptr), path_cost(0.0) {}

  Fast_RRTStarNode::Fast_RRTStarNode(world_coords x_in, world_coords y_in)
    : x(x_in), y(y_in), parent(nullptr), path_cost(0.0) {}

  Fast_RRTStarNode::Fast_RRTStarNode(Fast_RRTStarNode *node)
    : x(node->x), y(node->y), parent(node->parent), path_cost(node->path_cost) {}

  Fast_RRTStarNode::Fast_RRTStarNode(const Fast_RRTStarNode *const node)
    : x(node->x), y(node->y), parent(node->parent), path_cost(node->path_cost) {}

  void Fast_RRTStarNode::updateParentAndCost(Fast_RRTStarNode *node_parent)
  {
    this->parent = node_parent;
    this->path_cost = node_parent->path_cost + this->getConnectionCost(node_parent);

    return;
  }

  void Fast_RRTStarNode::updateParentAndCost(Fast_RRTStarNode *node_parent, world_coords path_cost)
  {
    this->parent = node_parent;
    this->path_cost = path_cost;

    return;
  }

  world_coords Fast_RRTStarNode::getConnectionCost(Fast_RRTStarNode *node_connect)
  {
    return Fast_RRTStarHelper::distance(node_connect, this);
  }

  bool Fast_RRTStarNode::operator==(const Fast_RRTStarNode node)
  {
    if (this->x == node.x && this->y == node.y)
    {
      return true;
    }

    return false;
  }

  bool Fast_RRTStarNode::operator!=(const Fast_RRTStarNode node)
  {
    return !(*this == node);
  }
}