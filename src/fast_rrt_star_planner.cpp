#include "fast_rrt_star_planner.h"

#include <pluginlib/class_list_macros.h>
#include <stdlib.h>
#include <limits>
#include <random>

PLUGINLIB_EXPORT_CLASS(fast_rrt_star_planner::Fast_RRTStarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace fast_rrt_star_planner
{
  Fast_RRTStarPlanner::Fast_RRTStarPlanner()
    : costmap_ros_(nullptr), initialized_(false) {}

  Fast_RRTStarPlanner::Fast_RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(nullptr), initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  void Fast_RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (initialized_ == false)
    {
      costmap_ros_ = costmap_ros;			        ///< initialize the costmap_ros_ attribute to the parameter
      costmap_ = costmap_ros_->getCostmap();  ///< get the costmap_ from costmap_ros_
      costmap_model_ = new base_local_planner::CostmapModel(*costmap_);
      resolution_ = costmap_->getResolution();
      origin_x_ = costmap_->getOriginX();
      origin_y_ = costmap_->getOriginY();
      size_x_ = costmap_->getSizeInMetersX();
      size_y_ = costmap_->getSizeInMetersY();

      int lethal_cost_int;

      /**
       * Initialize planner parameters from parameter server
       * Reference: http://wiki.ros.org/roscpp/Overview/Parameter%20Server
       */
      ros::NodeHandle nh("~/" + name);
      nh.param("max_num_iter", max_num_iter_, 500);
      nh.param("min_num_iter", min_num_iter_, 40);
      nh.param("world_near_radius", world_near_radius_, 2.0);
      nh.param("world_dist_dichotomy", world_dist_dichotomy_, 0.1);
      nh.param("world_expand_dist", world_expand_dist_, 2.5);
      nh.param("world_goal_accessible_dist", world_goal_accessible_dist_, 0.5);
      nh.param("goal_frequency", goal_frequency_, 0.2);
      nh.param("lethal_cost", lethal_cost_int, 253);

      lethal_cost_ = static_cast<map_cost>(lethal_cost_int);

      frame_id_ = costmap_ros->getGlobalFrameID();
      plan_pub_ = nh.advertise<nav_msgs::Path>("plan", 1);

      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  bool Fast_RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                 const geometry_msgs::PoseStamped &goal,
                                 std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (initialized_ == false)
    {
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    /**
     * Get updated costmap
     */
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if (start.header.frame_id != frame_id_)
    {
      ROS_ERROR("This planner as configured will only accept start poses in the %s frame, but a start was sent in the %s frame.",
                frame_id_.c_str(), start.header.frame_id.c_str());
    }

    if (goal.header.frame_id != frame_id_)
    {
      ROS_ERROR("This planner as configured will only accept goal poses in the %s frame, but a goal was sent in the %s frame.",
                frame_id_.c_str(), goal.header.frame_id.c_str());
    }

    ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    /**
     * Reset global plan variables
     * 
     */
    path_found_ = false;
    best_path_cost_ = numeric_limits<double>::max();
    node_best_path_end_ = nullptr;

    /**
     * Create nodes for start and goal poses
     * These are constant pointers to a const, so neither the pointer nor the values it points to can change
     */
    const Fast_RRTStarNode *const node_start = new Fast_RRTStarNode(start.pose.position.x, start.pose.position.y);
    const Fast_RRTStarNode *const node_goal = new Fast_RRTStarNode(goal.pose.position.x, goal.pose.position.y);

    /**
     * Initialize node vector with start node
     */
    Fast_RRTStarNode *node_start_copy = new Fast_RRTStarNode(node_start);
    node_vec_.push_back(node_start_copy);

    /**
     * For loop over maximum number of iterations
     */
    for (int i = 0; i < max_num_iter_; ++i)
    {
      /**
       * Find a random unoccupied node
       */
      Fast_RRTStarNode *node_rand = sampleFree(node_goal); ///< node's parent is not defined yet

      /**
       * nearest() to find index of node in existing tree
       *       that is closest to random position from
       *       SampleFree()
       */
      size_t node_nearest_ind = nearest(node_rand);
      Fast_RRTStarNode *node_nearest = node_vec_[node_nearest_ind];

      /**
       * Expand tree from node_nearest in direction of node_rand
       * until distance limit is reached
       */
      steer(node_nearest, node_rand, node_goal);

      /**
       * Check if straight line path has collision with obstacles
       */
      if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_nearest, node_rand) == true)
      {
        /**
         * near() to find all nodes in existing tree
         *       that are within R_near of random node
         */
        vector<size_t> near_nodes_list = near(node_rand); ///< Used by rewire()

        /**
         * findReachest() to find reachest node based
         *       on nearest node and random node and map
         */
        Fast_RRTStarNode *node_reachest = findReachest(node_nearest, node_rand);

        /**
         * createNode() to create new node to connect
         *       random node to nearest node using D_dichotomy
         *       and reachest node and random node and map
         */
        Fast_RRTStarNode *node_create = createNode(node_reachest, node_rand);

        /**
         * Check if node was able to be created
         */
        if (node_create != nullptr)
        {
          /**
           * If node was created, add created node and random node to tree
           * ! NOTE: Make sure node_rand is appended AFTER node_create
           *         in node_vec_ so it is the last element
           */
          if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_create, node_reachest->parent) == false)
          {
            ROS_WARN("Missed collision check b/w node_create and node_reachest->parent");
          }

          node_create->updateParentAndCost(node_reachest->parent);

          if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_rand, node_create) == false)
          {
            ROS_WARN("Missed collision check b/w node_rand and node_create");
          }

          node_rand->updateParentAndCost(node_create);

          node_vec_.push_back(node_create);
          node_vec_.push_back(node_rand);
        }
        else
        {
          /**
           * If node was not created, add only random node to tree
           */
          if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_rand, node_reachest) == false)
          {
            ROS_WARN("Missed collision check b/w node_rand and node_reachest");
          }

          node_rand->updateParentAndCost(node_reachest);

          node_vec_.push_back(node_rand);
        }

        /**
         * Check if initial path was found
         * Compute distance between goal location and node_vec_.back() location
         */
        if (initialPathFound(node_goal) == true)
        {
          path_found_ = true;
          double current_path_cost = node_vec_.back()->path_cost;

          if (current_path_cost < best_path_cost_)
          {
            best_path_cost_ = current_path_cost;
            node_best_path_end_ = node_vec_.back();
          }

          if (i > min_num_iter_)
          {
            /**
             * Exit for loop early if a valid plan was found
             * after the minimum number of iterations
             */
            break;
          }
        }

        /**
         * rewire() to improve path cost
         */
        rewire(node_rand, near_nodes_list);
      }
      else
      {
        delete node_rand;
      }
    }

    if (path_found_ == true)
    {
      ROS_INFO("PATH FOUND");
      constructPlan(start, goal, plan, node_goal);
    }
    else
    {
      ROS_INFO("NO PATH FOUND");
    }

    /**
     * Cleanup pointers when done
     */
    delete node_start;
    delete node_goal;
    for (size_t i = 0; i < node_vec_.size(); ++i)
    {
      delete node_vec_[i];
    }
    node_vec_.clear();

    return path_found_;
  }

  Fast_RRTStarNode *Fast_RRTStarPlanner::sampleFree(const Fast_RRTStarNode *const node_goal)
  {
    /**
     * Random sampling setup
     */
    std::random_device rd;  ///< Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); ///< Standard mersenne_twister_engine seeded with rd()

    /**
     * Sample between 0 and 1
     */
    uniform_real_distribution<double> rand_is_goal_dist(0, 1);
    double rand_is_goal_val = rand_is_goal_dist(gen);

    /**
     * If sample is between 0 and goal_frequency, set node_rand to node_goal
     */
    if (rand_is_goal_val < goal_frequency_)
    {
      Fast_RRTStarNode *node_rand = new Fast_RRTStarNode(node_goal);

      return node_rand;
    }

    /**
     * Otherwise do stuff below
     */
    Fast_RRTStarNode *node_rand = new Fast_RRTStarNode();

    /**
     * Get lower and upper bounds of costmap in world coordinates
     */
    world_coords rand_wx_lb = origin_x_;
    world_coords rand_wx_ub = origin_x_ + size_x_;
    world_coords rand_wy_lb = origin_y_;
    world_coords rand_wy_ub = origin_y_ + size_y_;

    /**
     * Create uniform distributions between lower and upper bound for x and y
     */
    uniform_real_distribution<world_coords> rand_wx_dist(rand_wx_lb, rand_wx_ub);
    uniform_real_distribution<world_coords> rand_wy_dist(rand_wy_lb, rand_wy_ub);

    /**
     * Sample from uniform distributions with random engine
     */
    world_coords rand_wx = rand_wx_dist(gen);
    world_coords rand_wy = rand_wy_dist(gen);

    /**
     * Update node_rand with randomly generated position
     */
    node_rand->x = rand_wx;
    node_rand->y = rand_wy;

    return node_rand;
  }

  size_t Fast_RRTStarPlanner::nearest(Fast_RRTStarNode *node_rand)
  {
    /**
     * Set initial index and world coordinate distance of nearest node
     */
    size_t nearest_node_ind = 0;
    world_coords nearest_node_cost = std::numeric_limits<world_coords>::max();

    /**
     * Search through node_vec_ to find closest node (based on connection cost)
     */
    for (size_t i = 0; i < node_vec_.size(); ++i)
    {
      world_coords candidate_cost = node_rand->getConnectionCost(node_vec_[i]);

      /**
       * Update nearest node index and nearest node distance
       * if candidate node is closer than previous closest node
       */
      if (candidate_cost < nearest_node_cost)
      {
        nearest_node_cost = candidate_cost;
        nearest_node_ind = i;
      }
    }

    return nearest_node_ind;
  }

  vector<size_t> Fast_RRTStarPlanner::near(Fast_RRTStarNode *node_rand)
  {
    /**
     * Create vector to hold indexes of near nodes
     */
    vector<size_t> near_node_idxs;

    /**
     * Loop through node_vec_
     */
    for (size_t i = 0; i < node_vec_.size(); ++i)
    {
      /**
       * Check if distance between node_rand and node in node_vec_
       * is less than the near radius
       */
      if (Fast_RRTStarHelper::distance(node_rand, node_vec_[i]) < world_near_radius_)
      {
        /**
         * Add near node index to near_node_idxs vector
         */
        near_node_idxs.push_back(i);
      }
    }

    return near_node_idxs;
  }

  Fast_RRTStarNode *Fast_RRTStarPlanner::findReachest(Fast_RRTStarNode *node_nearest, Fast_RRTStarNode *node_rand)
  {
    /**
     * Set new node to node_nearest
     */
    Fast_RRTStarNode *node_reachest = new Fast_RRTStarNode(node_nearest);

    /**
     * While not back at start node, search through node_nearest parents
     * to find the furthest collision free node from node_rand
     * grab start node, first pointer in node_vec
     */
    Fast_RRTStarNode *node_start = node_vec_.front();
    while (*node_start != *node_reachest)
    {
      if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_rand, node_reachest->parent) == true)
      {
        /**
         * Create temporary pointer to parent of node_reachest
         */
        Fast_RRTStarNode *node_reachest_parent = node_reachest->parent;
        
        /**
         * Delete old node_reachest
         */
        delete node_reachest;
        
        /**
         * Create new node_reachest as copy of old node_reachest parent
         */
        node_reachest = new Fast_RRTStarNode(node_reachest_parent);
      }
      else
      {
        break;
      }
    }

    return node_reachest;
  }

  Fast_RRTStarNode *Fast_RRTStarPlanner::createNode(Fast_RRTStarNode *node_reachest, Fast_RRTStarNode *node_rand)
  {
    Fast_RRTStarNode *node_allow = new Fast_RRTStarNode(node_reachest);
    Fast_RRTStarNode *node_start = node_vec_.front();
    Fast_RRTStarNode *node_mid = new Fast_RRTStarNode();
    Fast_RRTStarNode *node_forbid = nullptr;

    if (*node_start != *node_reachest)
    {
      node_forbid = new Fast_RRTStarNode(node_reachest->parent);

      /**
       * while allow/forbid nodes distance > dichotomy distance
       */
      while (Fast_RRTStarHelper::distance(node_allow, node_forbid) > world_dist_dichotomy_)
      {
        node_mid->x = (node_allow->x + node_forbid->x) / 2.0;
        node_mid->y = (node_allow->y + node_forbid->y) / 2.0;

        if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_rand, node_mid) == true)
        {
          node_allow->x = node_mid->x;
          node_allow->y = node_mid->y;
        }
        else
        {
          node_forbid->x = node_mid->x;
          node_forbid->y = node_mid->y;
        }
      }

      node_forbid->x = node_rand->x;
      node_forbid->y = node_rand->y;

      while (Fast_RRTStarHelper::distance(node_allow, node_forbid) > world_dist_dichotomy_)
      {
        node_mid->x = (node_allow->x + node_forbid->x) / 2.0;
        node_mid->y = (node_allow->y + node_forbid->y) / 2.0;

        if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_mid, node_reachest->parent) == true)
        {
          node_allow->x = node_mid->x;
          node_allow->y = node_mid->y;
        }
        else
        {
          node_forbid->x = node_mid->x;
          node_forbid->y = node_mid->y;
        }
      }
    }

    Fast_RRTStarNode *node_create = new Fast_RRTStarNode();
    if (*node_allow != *node_reachest)
    {
      node_create->x = node_allow->x;
      node_create->y = node_allow->y;
    }
    else
    {
      node_create = nullptr;
    }

    delete node_allow;
    delete node_mid;
    delete node_forbid;

    return node_create;
  }

  void Fast_RRTStarPlanner::rewire(Fast_RRTStarNode *node_rand, const vector<size_t> &near_nodes_list)
  {
    /**
     * for each node in near
     */
    for (size_t i = 0; i < near_nodes_list.size(); ++i)
    {
      size_t node_near_index_i = near_nodes_list[i];
      Fast_RRTStarNode *node_near_i = node_vec_[node_near_index_i];

      world_coords rand_rewire_cost = node_rand->path_cost + node_near_i->getConnectionCost(node_rand);
      if (node_near_i->path_cost > rand_rewire_cost)
      {
        if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_near_i, node_rand) == true)
        {
          node_near_i->updateParentAndCost(node_rand, rand_rewire_cost);
        }
      }
    }
  }

  bool Fast_RRTStarPlanner::initialPathFound(const Fast_RRTStarNode *const node_goal)
  {
    Fast_RRTStarNode *node_goal_copy = new Fast_RRTStarNode(node_goal);
    Fast_RRTStarNode *node_last = node_vec_.back();
    world_coords dist_to_goal = Fast_RRTStarHelper::distance(node_last, node_goal_copy);

    bool goal_dist_threshold_met = dist_to_goal < world_goal_accessible_dist_;
    bool last_to_goal_collision_free = Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_last, node_goal_copy);
    bool path_collision_free = true;

    Fast_RRTStarNode *node_n = node_last;

    /**
     * Verify that path is still collision free even if map has changed
     */
    while (node_n->parent != nullptr)
    {
      if (!Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_n, node_n->parent) == true)
      {
        path_collision_free = false;
        break;
      }

      node_n = node_n->parent;
    }

    delete node_goal_copy;

    return goal_dist_threshold_met && last_to_goal_collision_free && path_collision_free;
  }

  void Fast_RRTStarPlanner::constructPlan(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal,
                                      vector<geometry_msgs::PoseStamped> &plan,
                                      const Fast_RRTStarNode *const node_goal)
  {
    /**
     * Empty plan vector
     */
    plan.clear();
    ros::Time plan_time = ros::Time::now();

    /**
     * Create a node for the goal position
     */
    Fast_RRTStarNode *node_goal_copy = new Fast_RRTStarNode(node_goal);

    if (Fast_RRTStarHelper::isFootprintPathCollisionFree(costmap_, costmap_ros_, lethal_cost_, node_goal_copy, node_best_path_end_) == false)
    {
      ROS_WARN("Missed collision check b/w node_goal_copy and node_best_path_end_");
    }

    node_goal_copy->updateParentAndCost(node_best_path_end_);
    node_vec_.push_back(node_goal_copy);

    /**
     * Initialize node_n to goal node
     */
    Fast_RRTStarNode *node_n = node_vec_.back();
    bool isGoalNode = true;
    pose_value_type zVal = 0.0; ///< Z-axis is ignored

    /**
     * Go through parents of node until hitting start_node
     */
    while (node_n->parent != nullptr)
    {
      /**
       * Initialize new stamped pose
       */
      geometry_msgs::PoseStamped parent_pose_n;

      /**
       * Plan headers
       */
      parent_pose_n.header.stamp = plan_time;
      parent_pose_n.header.frame_id = frame_id_;

      /**
       * Set position
       */
      parent_pose_n.pose.position.x = node_n->parent->x;
      parent_pose_n.pose.position.y = node_n->parent->y;
      parent_pose_n.pose.position.z = zVal;

      /**
       * Set orientation
       * ! NOTE: Assuming FLU (x: Front, y: Left, z: Up) coordinate frame
       *         This means 0 yaw points front
       *         Positive yaw is CCW and negative yaw is CW.
       */
      world_coords start_x = node_n->parent->x;
      world_coords start_y = node_n->parent->y;
      world_coords end_x = node_n->x;
      world_coords end_y = node_n->y;
      world_coords dx = end_x - start_x;
      world_coords dy = end_y - start_y;

      pose_value_type yaw_in_rad = -atan2(dx, dy);
      tf::Quaternion parent_pose_n_quat = tf::createQuaternionFromYaw(yaw_in_rad);

      parent_pose_n.pose.orientation.x = parent_pose_n_quat.x();
      parent_pose_n.pose.orientation.y = parent_pose_n_quat.y();
      parent_pose_n.pose.orientation.z = parent_pose_n_quat.z();
      parent_pose_n.pose.orientation.w = parent_pose_n_quat.w();

      /**
       * Goal node re-uses orientation of parent, but updates position
       * NOTE: Make sure to push back goal pose first so reverse
       *       ordering of poses is correct
       */
      if (isGoalNode == true)
      {
        geometry_msgs::PoseStamped pose_n = parent_pose_n;

        /**
         * Set position
         */
        pose_n.pose.position.x = node_n->x;
        pose_n.pose.position.y = node_n->y;

        /**
         * Set orientation
         */
        pose_n.pose.orientation = goal.pose.orientation;

        /**
         * Add pose of goal node to plan
         */
        plan.push_back(pose_n);

        isGoalNode = false;
      }

      /**
       * Add pose of parent node to plan
       */
      plan.push_back(parent_pose_n);

      /**
       * Update node_n to its parent
       */
      node_n = node_n->parent;
    }

    /**
     * Reverse order of plan so pose associated with start_node is first
     * and pose associated with goal_node is last
     */
    reverse(plan.begin(), plan.end());

    return;
  }

  void Fast_RRTStarPlanner::steer(Fast_RRTStarNode *node_nearest, Fast_RRTStarNode *node_rand, const Fast_RRTStarNode *const node_goal)
  {
    /**
     * Calculate dist of vector from node_nearest to node_rand
     */
    world_coords dist = Fast_RRTStarHelper::distance(node_nearest, node_rand);

    /**
     * If node_rand is within expand dist, keep node_rand as is
     */
    if (dist < world_expand_dist_)
    {
      node_rand->updateParentAndCost(node_nearest);
      return;
    }

    double angle = Fast_RRTStarHelper::angle(node_nearest, node_rand);

    /**
     * Re-calculate x/y components of vector based on new distance
     */
    node_rand->x = node_nearest->x + world_expand_dist_ * cos(angle);
    node_rand->y = node_nearest->y + world_expand_dist_ * sin(angle);

    return;
  }
};
