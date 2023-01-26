/**
 * @brief This implementation largely follows what is described in ROS documentation:
 *        http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
 */
#ifndef FAST_RRT_STAR_PLANNER_H
#define FAST_RRT_STAR_PLANNER_H

#include "fast_rrt_star_helper.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;
using pose_value_type = double;

namespace fast_rrt_star_planner
{
  /**
   * Main class for Fast-RRT* algorithm that extends move_base's BaseGlobalPlanner.
   * Takes a starting location, goal location, and map of environment as input
   * and generates a set of poses for local planner to follow to get from
   * start location to goal location.
   *        
   * NOTE on units and variable names:
   * - map or m: Map coordinates (i.e., cell index)
   * - world or w: World coordinates (i.e., meters)
   */
  class Fast_RRTStarPlanner : public nav_core::BaseGlobalPlanner
  {
  private:
    /**
     * @brief Private Variables
     */
    bool initialized_;                                ///< Flag indicating if planner is initialized
    bool path_found_;                                 ///< Flag indicating if planner found a path
    world_coords best_path_cost_;                     ///< Track the shortest path distance found so far
    Fast_RRTStarNode *node_best_path_end_;                ///< Pointer to last node in best path
    base_local_planner::CostmapModel *costmap_model_; ///< Used by planner to check for collisions
    costmap_2d::Costmap2DROS *costmap_ros_;           ///< ROS 2D Costmap
    costmap_2d::Costmap2D *costmap_;                  ///< 2D Costmap
    std::vector<Fast_RRTStarNode *> node_vec_;            ///< Vector of planning nodes pointers
    world_coords resolution_;                         ///< Costmap resolution (size of grid cell in world coordinates)
    world_coords origin_x_;                           ///< x coordinate of costmap origin
    world_coords origin_y_;                           ///< y coordinate of costmap origin
    world_coords size_x_;                             ///< Size of costmap's x axis in world coordinates
    world_coords size_y_;                             ///< Size of costmap's y axis in world coordinates

    /**
     * @brief Tunable Parameters
     */
    int max_num_iter_;                        ///< Maximum number of iterations
    int min_num_iter_;                        ///< Minimum number of iterations
    world_coords world_near_radius_;          ///< Search radius (in world coordinates)
    world_coords world_dist_dichotomy_;       ///< Dichotomy distance (in world coordinates), the minimum
                                              ///< allowable distance between nodes created in CreateNode
    world_coords world_goal_accessible_dist_; ///< Threshold for considering goal position to be
                                              ///< accessible from current position
    world_coords world_expand_dist_;          ///< Maximum distance to expand tree when adding new nodes
    double goal_frequency_;                   ///< Frequency of random node being set to goal node
    map_cost lethal_cost_;                    ///< Cost at which to consider a cell inaccessible

    /**
     * @brief Randomly samples free space in costmap to generate a new node
     * 
     * @param node_goal Fixed goal node
     * @return Fast_RRTStarNode* Pointer to randomly sampled node
     */
    Fast_RRTStarNode *sampleFree(const Fast_RRTStarNode *const node_goal);

    /**
     * @brief Find the index of node in existing tree that is closest to node_rand
     * 
     * @param node_rand Randomly sampled node
     * @return size_t Index of node in existing tree that is nearest to node_rand 
     */
    size_t nearest(Fast_RRTStarNode *node_rand);

    /**
     * @brief Find all nodes (as indices in near_vec_) in existing tree
     *        that are within world_near_radius_ of random node
     * 
     * @param node_rand Randomly sampled node
     * @return std::vector<size_t> Vector of node indices that are within world_near_radius_ of node_rand
     */
    std::vector<size_t> near(Fast_RRTStarNode *node_rand);

    /**
     * @brief Find furthest possible away node from node_rand that is collision free.
     *        Only search through ancestors of node_nearest.
     * 
     * @param node_nearest Nearest node in distance to randomly sampled node node_rand
     * @param node_rand Randomly sampled node
     * @return Fast_RRTStarNode* Pointer to node_reachest
     */
    Fast_RRTStarNode *findReachest(Fast_RRTStarNode *node_nearest, Fast_RRTStarNode *node_rand);

    /**
     * @brief Create a new node to connect node_rand to node_nearest using
     *        world_dist_dichotomy_, node_reachest, node_rand, and costmap_.
     * 
     * @param node_reachest Eldest ancestor node of node_rand that is collision free to node_rand
     * @param node_rand Randomly sampled node
     * @return Fast_RRTStarNode* Pointer to node_create
     */
    Fast_RRTStarNode *createNode(Fast_RRTStarNode *node_reachest, Fast_RRTStarNode *node_rand);

    /**
     * @brief Check if latest node added to node_vec_ is within threshold
     *        distance of goal position
     * 
     * @param node_goal Fixed goal node
     * @return true if tree is close enough to node_goal
     * @return false if tree is not close enough to node_goal
     */
    bool initialPathFound(const Fast_RRTStarNode *const node_goal);

    /**
     * @brief Changes the connection relationships of tree vertices.
     *        If replacing the parent node of node_near by node_rand reduces path from start to node_near, then do so.
     *        Returns the re-ordered node vector
     * 
     * @param node_rand Randomly sampled node
     * @param near_nodes_list List of nodes within specified distance limit of node_rand
     */
    void rewire(Fast_RRTStarNode *node_rand, const std::vector<size_t> &near_nodes_list);

    /**
     * @brief Construct the vector of stamped poses from linked list of
     *        Fast-RRT* node pointers
     * 
     * @param start Time stamped pose of start position
     * @param goal Time stamped pose of goal position
     * @param plan List of stamped poses, starting with start node and ending with goal
     * @param node_goal Fixed goal node
     */
    void constructPlan(const geometry_msgs::PoseStamped &start,
                       const geometry_msgs::PoseStamped &goal,
                       std::vector<geometry_msgs::PoseStamped> &plan,
                       const Fast_RRTStarNode *const node_goal);

    /**
     * @brief Update node_rand so tree extends from node_nearest in direction of original
     *        node_rand until it reaches an occupied or unknown cell
     * 
     * @param node_nearest Node nearest to sampled node
     * @param node_rand Randomly sampled node
     * @param node_goal Fixed goal node
     */
    void steer(Fast_RRTStarNode *node_nearest, Fast_RRTStarNode *node_rand, const Fast_RRTStarNode *const node_goal);

  public:
    /**
     * @brief Public Variables
     */
    ros::Publisher plan_pub_;
    std::string frame_id_;

    /**
     * @brief Construct a new Fast_RRTStarPlanner object.
     *        Initializes the planner attributes with the default values.
     */
    Fast_RRTStarPlanner();

    /**
     * @brief Construct a new Fast_RRTStarPlanner object.
     *        Used to initialize the costmap, that is the map that will be used for planning (costmap_ros),
     *        and the name of the planner (name).
     * 
     * @param name Name of the planner
     * @param costmap_ros ROS version of costmap
     */
    Fast_RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /**
	   * @brief Overridden class from interface nav_core::BaseGlobalPlanner
     *        An initialization function for the BaseGlobalPlanner, which initializes the costmap, 
	   *        that is the map that will be used for planning (costmap_ros), and the name of the planner (name).
     * 
     * @param name Name of planner
     * @param costmap_ros ROS version of costmap
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
	
    /**
     * @brief Overridden class from interface nav_core::BaseGlobalPlanner
     * 
	   * @param start Pose of starting position
	   * @param goal Pose of end position
	   * @param plan Vector of timestamped poses
     * @return true if planner could find a path from start to goal
     * @return false if planner could not find a path from start to goal
     */
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
  };
};

#endif /* FAST_RRT_STAR_PLANNER_H */