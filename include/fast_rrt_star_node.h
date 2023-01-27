#ifndef FAST_RRT_STAR_NODE_H
#define FAST_RRT_STAR_NODE_H

/**
 * Create type alias for world coordinates
 */
using world_coords = double;

namespace fast_rrt_star_planner
{
  /**
   * @brief Fast_RRTStarNode class used to track nodes of path planning tree.
   */
  class Fast_RRTStarNode
  {
  public:
    /**
     * @brief Public Variables
     */
    world_coords x;         ///< x coordinate of node
    world_coords y;         ///< y coordinate of node
    world_coords path_cost; ///< Cumulative cost from node_start to current node
    Fast_RRTStarNode *parent;   ///< Pointer to parent of current node

    /**
     * @brief Construct a new Fast_RRTStarNode object with default values
     */
    Fast_RRTStarNode();

    /**
     * @brief Construct a new Fast_RRTStarNode object with given position
     * 
     * @param x_in X component of node in world coordinates
     * @param y_in Y component of node in world coordinates
     */
    Fast_RRTStarNode(world_coords x_in, world_coords y_in);

    /**
     * @brief Construct a new Fast_RRTStarNode object as a deep copy of pointer to input node
     * 
     * @param node Pointer to input node to deep copy
     */
    Fast_RRTStarNode(Fast_RRTStarNode *node);

    /**
     * @brief Construct a new Fast_RRTStarNode object as a deep copy of constant pointer to constant input node
     * 
     * @param node Constant pointer to constant input node to deep copy
     */
    Fast_RRTStarNode(const Fast_RRTStarNode *const node);

    /** 
     * @brief "Connects" node to node_parent by updating parent pointer and path_cost
     *
     * @param node_parent Desired node being assigned as parent
     */
    void updateParentAndCost(Fast_RRTStarNode *node_parent);

    /** 
     * @brief "Connects" this node to node_parent by updating parent pointer and path_cost
     *        without computing distance between node and node_parent internally,
     *        just uses the provided cost (reason: saves re-computation)
     *  
     * @param node_parent Desired node being assigned as parent
     * @param path_cost Path cost (i.e. distance) between current node and node_parent
     */
    void updateParentAndCost(Fast_RRTStarNode *node_parent, world_coords path_cost);

    /** 
     * @brief Compute cost of connecting this node to node_connect (hypothetical parent) 
     *
     * @param node_connect Node chosen to calculate cost to
     * @return world_coords Cost of path between the two nodes
     */
    world_coords getConnectionCost(Fast_RRTStarNode *node_connect);

    /**
     * @brief Overloaded equality operator to compare two Fast_RRTStarNodes
     * 
     * @param node Node to compare to current node for equality
     * @return true If properties of current and input node are the same
     * @return false If properties of current and input node are not the same
     */
    bool operator==(const Fast_RRTStarNode node);

    /**
     * @brief Overloaded inequality operator to compare two Fast_RRTStarNodes
     * 
     * @param node Node to compare to current node for inequality
     * @return true If properties of current and input node are not the same
     * @return false If properties of current and input node are the same
     */
    bool operator!=(const Fast_RRTStarNode node);
  };
}

#endif /* FAST_RRT_STAR_NODE_H */