#ifndef AEPLANNER_H
#define AEPLANNER_H

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <eigen3/Eigen/Dense>

#include <kdtree/kdtree.h>

#include <aeplanner/data_structures.h>
#include <aeplanner/param.h>
#include <aeplanner/Reevaluate.h>

#include <aeplanner/aeplanner_viz.h>
#include <visualization_msgs/MarkerArray.h>

#include <aeplanner/aeplannerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <pigain/Node.h>
#include <pigain/Query.h>
#include <pigain/BestNode.h>

#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// DAEP
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <aeplanner/kalman.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <iostream>
#include <tuple>
#include <string>
#include <pigain/QueryDFM.h>
#include <pigain/Score.h>
#include <tree_identifier/TreeDetectionArray.h>

// synch
#include <mutex>
#include <map>


namespace aeplanner
{
class AEPlanner
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<aeplanner::aeplannerAction> as_;

  Params params_;

  // Current state of agent (x, y, z, yaw)
  Eigen::Vector4d current_state_;
  bool current_state_initialized_;
  bool dynamic_mode_;

  // Keep track of the best node and its score
  RRTNode* best_node_;
  RRTNode* best_branch_root_;

  std::shared_ptr<octomap::OcTree> ot_;

  // kd tree for finding nearest neighbours
  kdtree* kd_tree_;

  // Subscribers
  ros::Subscriber octomap_sub_;
  ros::Subscriber agent_pose_sub_;
  ros::Subscriber tree_confirmed_sub_;
  ros::Subscriber tree_candidate_sub_;

  // DAEP
  ros::Subscriber human_sub_;
  ros::Publisher pred_marker_pub_;
  ros::Publisher human_marker_pub_;
  ros::Publisher covariance_marker_pub_;
  ros::Publisher best_marker_pub_;
  ros::Publisher ghost_marker_pub_;
  ros::Publisher static_rays_marker_pub_;
  ros::Publisher dynamic_rays_marker_pub_;
  ros::Publisher old_path_marker_pub_;
  // Publishers
  ros::Publisher rrt_marker_pub_;
  ros::Publisher gain_pub_;

  // Services
  ros::ServiceClient best_node_client_;
  ros::ServiceClient gp_query_client_;
  ros::ServiceServer reevaluate_server_;
  ros::ServiceClient nn_yaw_query_client_;
  ros::ServiceClient dfm_client_;
  std::string tree_guidance_log_path_;
  bool tree_guidance_log_ready_;

  // DAEP

  //Global variables
  std::map<std::string, std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> dynamic_objects;
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> predicted_data;
  std::mutex vecMutex;

  struct TreeTargetState
  {
    int id;
    Eigen::Vector3d position;
    double confidence;
    double fit_error;
    double diameter;
    int hits;
    ros::Time last_hit_update;
    bool confirmed;
  };

  struct TreeGainBreakdown
  {
    double score;
    int best_tree_id;
    double best_tree_distance;
    double best_tree_confidence;
    double best_tree_contribution;
    bool best_tree_confirmed;
    int total_trees;
    int considered_trees;
    int confirmed_considered;
    int candidate_considered;

    TreeGainBreakdown()
        : score(0.0)
        , best_tree_id(-1)
        , best_tree_distance(0.0)
        , best_tree_confidence(0.0)
        , best_tree_contribution(0.0)
        , best_tree_confirmed(false)
        , total_trees(0)
        , considered_trees(0)
        , confirmed_considered(0)
        , candidate_considered(0)
    {
    }
  };
  std::map<int, TreeTargetState> confirmed_trees_;
  std::map<int, TreeTargetState> candidate_trees_;
  mutable std::mutex tree_map_mutex_;

  void confirmedTreeMapCallback(const tree_identifier::TreeDetectionArray::ConstPtr& msg);
  void candidateTreeMapCallback(const tree_identifier::TreeDetectionArray::ConstPtr& msg);
  void updateTreeMap(const tree_identifier::TreeDetectionArray::ConstPtr& msg, bool confirmed);
  double treeInformationGain(const Eigen::Vector4d& state, double time_of_arrival) const;
  TreeGainBreakdown treeInformationGainBreakdown(const Eigen::Vector4d& state, double time_of_arrival) const;
  void applyTreeGuidanceToNode(RRTNode* node, double time_of_arrival, std::tuple<double, double, double>& gain_tuple);
  void initTreeGuidanceLog();
  void logTreeGuidanceDecision(int iteration, int actions_taken, const RRTNode* executed_node, const RRTNode* best_leaf, const std::string& planner);
  std::vector<TreeTargetState> snapshotTrees() const;

  std::vector<std::tuple<double, double, Eigen::MatrixXd>> createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices);
  bool willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts);

  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> KFpredictTrajectories();
  int getCovarianceIndex(double max_time_step, double time_step, double t);
  bool checkCollision(double t, 
                      Eigen::Vector4d point, 
                      std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                      std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);

  bool isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse);

  bool isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z);

  // Service server callback
  bool reevaluate(aeplanner::Reevaluate::Request& req,
                  aeplanner::Reevaluate::Response& res);

  // ---------------- Initialization ----------------
  RRTNode* initialize(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                              std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);
                              void initializeKDTreeWithPreviousBestBranch(RRTNode* root);

  std::pair<RRTNode*, bool> pathIsSafe(RRTNode* node, 
                          std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);
  void reevaluatePotentialInformationGainRecursive(RRTNode* node);

  // ---------------- Expand RRT Tree ----------------
  void expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses);

  Eigen::Vector4d sampleNewPoint();
  bool isInsideBoundaries(Eigen::Vector4d point);
  bool pointOnXYBoundaries(Eigen::Vector4d point);
  bool collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r);
  RRTNode* chooseParent(RRTNode* node, double l);
  void rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r, double r_os);
  Eigen::Vector4d restrictDistance(Eigen::Vector4d nearest, Eigen::Vector4d new_pos);

  std::tuple<double, double, double> getGain(RRTNode* node, double time_of_arrival);

  std::tuple<double, double, double> gainCubature(Eigen::Vector4d state, double time_of_arrival);

  // ---------------- Helpers ----------------
  //
  void publishEvaluatedNodesRecursive(RRTNode* node);

  geometry_msgs::Pose vecToPose(Eigen::Vector4d state);

  float CylTest_CapsFirst(const octomap::point3d& pt1, const octomap::point3d& pt2,
                          float lsq, float rsq, const octomap::point3d& pt);

  // ---------------- Frontier ----------------
  geometry_msgs::PoseArray getFrontiers();

public:
  AEPlanner(const ros::NodeHandle& nh);

  void execute(const aeplanner::aeplannerGoalConstPtr& goal);

  // DAEP
  void updateHumanPositions(const gazebo_msgs::ModelStates& model_states);

  void octomapCallback(const octomap_msgs::Octomap& msg);
  //void agentPoseCallback(const geometry_msgs::PoseStamped& msg);
  //void agentPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void agentPoseCallback(const geometry_msgs::PoseStamped& msg);

};

}  // namespace aeplanner

#endif
