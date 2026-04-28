#include <aeplanner/aeplanner.h>
#include <tf2/utils.h>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace
{
double clamp01(double v)
{
  if (v < 0.0)
    return 0.0;
  if (v > 1.0)
    return 1.0;
  return v;
}
} // namespace

namespace aeplanner
{
AEPlanner::AEPlanner(const ros::NodeHandle& nh)
  : nh_(nh)
  , as_(nh_, "make_plan", boost::bind(&AEPlanner::execute, this, _1), false)
  , octomap_sub_(nh_.subscribe("octomap", 1, &AEPlanner::octomapCallback, this))
  , agent_pose_sub_(nh_.subscribe("/pose", 1, &AEPlanner::agentPoseCallback, this)) 
  , human_sub_(nh_.subscribe("/gazebo/model_states", 1, &AEPlanner::updateHumanPositions, this)) // DAEP
  , rrt_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("rrtree", 1000))
  , pred_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("predicted_trajectory", 1000)) // DAEP
  , human_marker_pub_(nh_.advertise<visualization_msgs::Marker>("human_poses", 1000)) // DAEP
  , best_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("best_node", 1000)) // DAEP
  , covariance_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("cov_ellipse", 1000)) // DAEP
  , ghost_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("ghosts", 1000)) // DAEP
  , static_rays_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("static_rays", 1000)) // DAEP
  , dynamic_rays_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("dynamic_rays", 1000)) // DAEP
  , old_path_marker_pub_(nh_.advertise<visualization_msgs::MarkerArray>("old_path", 1000)) // DAEP
  , gain_pub_(nh_.advertise<pigain::Node>("gain_node", 1000))
  , gp_query_client_(nh_.serviceClient<pigain::Query>("gp_query_server"))
  , nn_yaw_query_client_(nh_.serviceClient<pigain::Query>("nn_yaw_query_server"))
  , reevaluate_server_(nh_.advertiseService("reevaluate", &AEPlanner::reevaluate, this))
  , best_node_client_(nh_.serviceClient<pigain::BestNode>("best_node_server"))
  , dfm_client_(nh_.serviceClient<pigain::QueryDFM>("dfm_query_server"))
  , current_state_initialized_(false)
  , ot_(NULL)
  , best_node_(NULL)
  , best_branch_root_(NULL)
  , dynamic_mode_(false)
  , tree_guidance_log_ready_(false)
{
  params_ = readParams();
  initTreeGuidanceLog();
  int experiment_seed = -1;
  const std::string ns = ros::this_node::getNamespace();
  if (ros::param::get(ns + "/experiment_seed", experiment_seed) && experiment_seed >= 0) {
    srand(static_cast<unsigned int>(experiment_seed));
    ROS_INFO_STREAM("AEPlanner random seed: " << experiment_seed);
  }

  if (params_.tree_guidance_enabled)
  {
    tree_confirmed_sub_ = nh_.subscribe(
        params_.tree_confirmed_topic, 1, &AEPlanner::confirmedTreeMapCallback, this);
    tree_candidate_sub_ = nh_.subscribe(
        params_.tree_candidate_topic, 1, &AEPlanner::candidateTreeMapCallback, this);
    ROS_INFO_STREAM("Tree guidance enabled. confirmed_topic=" << params_.tree_confirmed_topic
                    << " candidate_topic=" << params_.tree_candidate_topic
                    << " gain_weight=" << params_.tree_gain_weight);
  }
  else
  {
    ROS_INFO("Tree guidance disabled.");
  }
  as_.start();

  // Initialize kd-tree
  kd_tree_ = kd_create(3);
}

void AEPlanner::confirmedTreeMapCallback(const tree_identifier::TreeDetectionArray::ConstPtr& msg)
{
  updateTreeMap(msg, true);
}

void AEPlanner::candidateTreeMapCallback(const tree_identifier::TreeDetectionArray::ConstPtr& msg)
{
  updateTreeMap(msg, false);
}

void AEPlanner::updateTreeMap(const tree_identifier::TreeDetectionArray::ConstPtr& msg, bool confirmed)
{
  std::lock_guard<std::mutex> lock(tree_map_mutex_);
  std::map<int, TreeTargetState>& target_map = confirmed ? confirmed_trees_ : candidate_trees_;
  const ros::Time now = ros::Time::now();

  std::map<int, TreeTargetState> updated;
  for (std::vector<tree_identifier::TreeDetection>::const_iterator it = msg->detections.begin();
       it != msg->detections.end(); ++it)
  {
    const tree_identifier::TreeDetection& det = *it;
    const int id = static_cast<int>(det.id);
    if (id <= 0)
      continue;

    TreeTargetState state;
    state.id = id;
    state.position = Eigen::Vector3d(det.pose.position.x, det.pose.position.y, det.pose.position.z);
    state.confidence = clamp01(static_cast<double>(det.confidence));
    state.fit_error = std::max(0.0, static_cast<double>(det.fit_error));
    state.diameter = std::max(0.0, static_cast<double>(det.diameter));
    state.hits = std::max(0, static_cast<int>(det.cluster_points));
    state.confirmed = confirmed;
    state.last_hit_update = now;

    std::map<int, TreeTargetState>::const_iterator prev = target_map.find(id);
    if (prev != target_map.end())
    {
      if (state.hits <= prev->second.hits)
      {
        state.last_hit_update = prev->second.last_hit_update;
      }
    }

    updated[id] = state;
  }

  target_map.swap(updated);
}

std::vector<AEPlanner::TreeTargetState> AEPlanner::snapshotTrees() const
{
  std::lock_guard<std::mutex> lock(tree_map_mutex_);
  std::vector<TreeTargetState> trees;
  trees.reserve(confirmed_trees_.size() + candidate_trees_.size());

  for (std::map<int, TreeTargetState>::const_iterator it = confirmed_trees_.begin();
       it != confirmed_trees_.end(); ++it)
  {
    trees.push_back(it->second);
  }
  for (std::map<int, TreeTargetState>::const_iterator it = candidate_trees_.begin();
       it != candidate_trees_.end(); ++it)
  {
    trees.push_back(it->second);
  }
  return trees;
}

double AEPlanner::treeInformationGain(const Eigen::Vector4d& state, double time_of_arrival) const
{
  return treeInformationGainBreakdown(state, time_of_arrival).score;
}

AEPlanner::TreeGainBreakdown AEPlanner::treeInformationGainBreakdown(const Eigen::Vector4d& state, double time_of_arrival) const
{
  TreeGainBreakdown breakdown;
  if (!params_.tree_guidance_enabled)
    return breakdown;

  const std::vector<TreeTargetState> trees = snapshotTrees();
  breakdown.total_trees = static_cast<int>(trees.size());
  if (trees.empty())
    return breakdown;

  const double now_sec = ros::Time::now().toSec();
  const double min_range = std::max(0.0, params_.tree_view_min_range);
  const double max_range = std::max(min_range, params_.tree_view_max_range);
  const double pref_range = std::max(min_range, params_.tree_preferred_range);
  const double sigma = std::max(1e-6, params_.tree_range_sigma);
  const double fit_norm = std::max(1e-6, params_.tree_fit_error_norm);
  const double age_tau = std::max(1e-6, params_.tree_age_tau_sec);

  for (std::vector<TreeTargetState>::const_iterator it = trees.begin(); it != trees.end(); ++it)
  {
    const TreeTargetState& tree = *it;
    const double dx = state[0] - tree.position[0];
    const double dy = state[1] - tree.position[1];
    const double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy < min_range || dist_xy > max_range)
      continue;
    breakdown.considered_trees++;
    if (tree.confirmed)
      breakdown.confirmed_considered++;
    else
      breakdown.candidate_considered++;

    const double range_err = (dist_xy - pref_range) / sigma;
    const double range_weight = std::exp(-0.5 * range_err * range_err);

    const double conf_unc = clamp01(1.0 - tree.confidence);
    const double fit_unc = clamp01(tree.fit_error / fit_norm);
    const double age_sec = std::max(0.0, now_sec - tree.last_hit_update.toSec() + time_of_arrival);
    const double age_unc = 1.0 - std::exp(-age_sec / age_tau);

    double uncertainty =
        params_.tree_uncertainty_conf_weight * conf_unc +
        params_.tree_uncertainty_fit_weight * fit_unc +
        params_.tree_uncertainty_age_weight * age_unc;

    if (!tree.confirmed)
      uncertainty *= std::max(1.0, params_.tree_candidate_bonus);

    const double contribution = range_weight * std::max(0.0, uncertainty);
    breakdown.score += contribution;
    if (contribution > breakdown.best_tree_contribution)
    {
      breakdown.best_tree_id = tree.id;
      breakdown.best_tree_distance = dist_xy;
      breakdown.best_tree_confidence = tree.confidence;
      breakdown.best_tree_contribution = contribution;
      breakdown.best_tree_confirmed = tree.confirmed;
    }
  }

  return breakdown;
}

void AEPlanner::applyTreeGuidanceToNode(RRTNode* node, double time_of_arrival, std::tuple<double, double, double>& gain_tuple)
{
  if (!node)
    return;

  node->base_dynamic_gain_ = std::get<1>(gain_tuple);
  node->tree_gain_raw_ = 0.0;
  node->tree_gain_weighted_ = 0.0;
  node->tree_best_id_ = -1;
  node->tree_best_distance_ = 0.0;
  node->tree_best_confidence_ = 0.0;
  node->tree_best_contribution_ = 0.0;
  node->tree_best_confirmed_ = false;
  node->tree_total_count_ = 0;
  node->tree_considered_count_ = 0;
  node->tree_confirmed_considered_ = 0;
  node->tree_candidate_considered_ = 0;

  if (!params_.tree_guidance_enabled)
    return;

  const TreeGainBreakdown tree_gain = treeInformationGainBreakdown(node->state_, time_of_arrival);
  node->tree_gain_raw_ = tree_gain.score;
  node->tree_gain_weighted_ = params_.tree_gain_weight * tree_gain.score;
  node->tree_best_id_ = tree_gain.best_tree_id;
  node->tree_best_distance_ = tree_gain.best_tree_distance;
  node->tree_best_confidence_ = tree_gain.best_tree_confidence;
  node->tree_best_contribution_ = tree_gain.best_tree_contribution;
  node->tree_best_confirmed_ = tree_gain.best_tree_confirmed;
  node->tree_total_count_ = tree_gain.total_trees;
  node->tree_considered_count_ = tree_gain.considered_trees;
  node->tree_confirmed_considered_ = tree_gain.confirmed_considered;
  node->tree_candidate_considered_ = tree_gain.candidate_considered;

  std::get<1>(gain_tuple) += node->tree_gain_weighted_;
  ROS_DEBUG_STREAM_THROTTLE(
      1.0,
      "tree_guidance gain: base_dynamic=" << node->base_dynamic_gain_
      << " tree_gain=" << node->tree_gain_raw_
      << " weighted=" << node->tree_gain_weighted_
      << " final_dynamic=" << std::get<1>(gain_tuple)
      << " best_tree_id=" << node->tree_best_id_
      << " considered=" << node->tree_considered_count_);
}

void AEPlanner::initTreeGuidanceLog()
{
  const char* home = std::getenv("HOME");
  if (!home)
  {
    ROS_WARN("HOME is not set. Tree guidance decision log disabled.");
    return;
  }

  tree_guidance_log_path_ = std::string(home) + "/data/tree_guidance_waypoints.csv";
  std::ofstream out(tree_guidance_log_path_.c_str(), std::ios::out | std::ios::trunc);
  if (!out.is_open())
  {
    ROS_WARN_STREAM("Could not open tree guidance decision log: " << tree_guidance_log_path_);
    return;
  }

  out << "ros_time,iteration,actions_taken,planner,"
      << "goal_x,goal_y,goal_z,goal_yaw,"
      << "goal_static_gain,goal_base_dynamic_gain,goal_tree_gain_raw,goal_tree_gain_weighted,goal_dynamic_gain_final,goal_dynamic_score,goal_dfm_score,"
      << "goal_best_tree_id,goal_best_tree_distance_m,goal_best_tree_confidence,goal_best_tree_confirmed,goal_best_tree_contribution,"
      << "goal_tree_total_count,goal_tree_considered_count,goal_tree_confirmed_considered,goal_tree_candidate_considered,"
      << "leaf_x,leaf_y,leaf_z,leaf_yaw,"
      << "leaf_static_gain,leaf_base_dynamic_gain,leaf_tree_gain_raw,leaf_tree_gain_weighted,leaf_dynamic_gain_final,leaf_dynamic_score,leaf_dfm_score,"
      << "leaf_best_tree_id,leaf_best_tree_distance_m,leaf_best_tree_confidence,leaf_best_tree_confirmed,leaf_best_tree_contribution,"
      << "leaf_tree_total_count,leaf_tree_considered_count,leaf_tree_confirmed_considered,leaf_tree_candidate_considered,"
      << "tree_gain_weight,zero_gain"
      << std::endl;
  tree_guidance_log_ready_ = true;
  ROS_INFO_STREAM("Tree guidance decision log: " << tree_guidance_log_path_);
}

void AEPlanner::logTreeGuidanceDecision(int iteration, int actions_taken, const RRTNode* executed_node, const RRTNode* best_leaf, const std::string& planner)
{
  if (!tree_guidance_log_ready_ || !executed_node)
    return;

  std::ofstream out(tree_guidance_log_path_.c_str(), std::ios::out | std::ios::app);
  if (!out.is_open())
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "Could not append tree guidance decision log: " << tree_guidance_log_path_);
    return;
  }

  const RRTNode* leaf = best_leaf ? best_leaf : executed_node;
  out << std::fixed << std::setprecision(6)
      << ros::Time::now().toSec() << ","
      << iteration << ","
      << actions_taken << ","
      << planner << ","
      << executed_node->state_[0] << ","
      << executed_node->state_[1] << ","
      << executed_node->state_[2] << ","
      << executed_node->state_[3] << ","
      << executed_node->gain_ << ","
      << executed_node->base_dynamic_gain_ << ","
      << executed_node->tree_gain_raw_ << ","
      << executed_node->tree_gain_weighted_ << ","
      << executed_node->dynamic_gain_ << ","
      << executed_node->dynamic_score(params_.lambda, params_.zeta) << ","
      << executed_node->dfm_score_ << ","
      << executed_node->tree_best_id_ << ","
      << executed_node->tree_best_distance_ << ","
      << executed_node->tree_best_confidence_ << ","
      << (executed_node->tree_best_confirmed_ ? 1 : 0) << ","
      << executed_node->tree_best_contribution_ << ","
      << executed_node->tree_total_count_ << ","
      << executed_node->tree_considered_count_ << ","
      << executed_node->tree_confirmed_considered_ << ","
      << executed_node->tree_candidate_considered_ << ","
      << leaf->state_[0] << ","
      << leaf->state_[1] << ","
      << leaf->state_[2] << ","
      << leaf->state_[3] << ","
      << leaf->gain_ << ","
      << leaf->base_dynamic_gain_ << ","
      << leaf->tree_gain_raw_ << ","
      << leaf->tree_gain_weighted_ << ","
      << leaf->dynamic_gain_ << ","
      << leaf->dynamic_score(params_.lambda, params_.zeta) << ","
      << leaf->dfm_score_ << ","
      << leaf->tree_best_id_ << ","
      << leaf->tree_best_distance_ << ","
      << leaf->tree_best_confidence_ << ","
      << (leaf->tree_best_confirmed_ ? 1 : 0) << ","
      << leaf->tree_best_contribution_ << ","
      << leaf->tree_total_count_ << ","
      << leaf->tree_considered_count_ << ","
      << leaf->tree_confirmed_considered_ << ","
      << leaf->tree_candidate_considered_ << ","
      << params_.tree_gain_weight << ","
      << params_.zero_gain
      << std::endl;
}

/**
 * Subscribe on the gazebo/model_states topic to extract the positions of the 
 * dynamic obstacles. Add these to the dynamic_objects std::map and visualzie 
 * the ground truth in RViz.
*/
void AEPlanner::updateHumanPositions(const gazebo_msgs::ModelStates& model_states) {
    int human_id = 0;

    for (size_t i = 0; i < model_states.name.size(); ++i) {
        if (model_states.name[i].find("person_walking") != std::string::npos) {
            dynamic_mode_ = true;
            std::string model_name = model_states.name[i];
            geometry_msgs::Pose person_pose = model_states.pose[i];
            geometry_msgs::Twist person_twist = model_states.twist[i];
            dynamic_objects[model_name] = std::make_pair(person_pose, person_twist);
            visualizePose(human_marker_pub_, human_id, person_pose);
        }
        human_id++;
    }
} 

/**
 * From a vector of covariance matrices, compute a list of tuples that
 * represent each covariance circle.
 * 
 * Return: (major_lenght, minor_length, eigenvectors) 
*/
std::vector<std::tuple<double, double, Eigen::MatrixXd>> AEPlanner::createCovarianceEllipse(const std::vector<Eigen::MatrixXd>& cov_matrices)
{
  
  std::vector<std::tuple<double, double, Eigen::MatrixXd>> ellipses{};

  for(auto const& cov_matrix : cov_matrices)
  {
    //Extract the part concerning x,y
    Eigen::MatrixXd cov_matrix_xy = cov_matrix.block<2,2>(0,0);

    // Compute the eigenvalues and eigenvectors of the covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(cov_matrix_xy);
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();

    // Compute the length of the major and minor axes of the ellipse
    double major_length = std::sqrt(std::max(eigenvalues(0), eigenvalues(1)));
    double minor_length = std::sqrt(std::min(eigenvalues(0), eigenvalues(1)));

    ellipses.push_back(std::make_tuple(major_length, minor_length, eigenvectors));
  }
  return ellipses;
}


/**
* This function uses the Kalman Filter with the Constant Velocity
* motion model to predict the future trajectory of each dynamic obstacle.
* The returned data consists of the means and covariances for each dynamic obstacle in a trajectory.
*/
std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> AEPlanner::KFpredictTrajectories()
{
  std::vector<std::pair<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>, std::vector<Eigen::MatrixXd>>> kf_data{};

  //For each pedestrian, we need to build a new Kalman filter
  int n = 4; //Number of states
  int m = 2; //Number of states we measure

  double dt = params_.time_step;

  //Matrices for the Kalman filter
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  //Constant Velocity Model
  A <<  1, 0, dt, 0, 
        0, 1, 0, dt, 
        0, 0, 1, 0, 
        0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 1, 0, 0;

  //Reasonable covariance matrices
  Q << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  R << 0.01, 0,
       0, 0.01;
  
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1; 

  Eigen::VectorXd y(m);
  Eigen::VectorXd x0(n);
  Eigen::MatrixXd P0(n, n);
  double t = 0;
  int safety_iteration = 1; // Bounding covariance (1 second)
  
  for (const auto& dynamic_obstacle : dynamic_objects) {
      const std::string& key = dynamic_obstacle.first;
      const geometry_msgs::Pose& pose = dynamic_obstacle.second.first;
      const geometry_msgs::Twist& twist = dynamic_obstacle.second.second;

      // Create a new Kalman filter for each dynamic obstacle
      KalmanFilter kf(dt, A, C, Q, R, P);
      std::vector<double> xcoords{};
      std::vector<double> ycoords{};
      std::vector<double> zcoords{};
      std::vector<Eigen::MatrixXd> covariance_matrices(params_.KFiterations);

      // Initalize with the last measurement
      double xcoord = pose.position.x, ycoord = pose.position.y, zcoord = pose.position.z;
      double vx = twist.linear.x, vy = twist.linear.y;
      x0 << xcoord, ycoord, vx, vy;
      
      //Initial covariance
      P0 << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1; 

      kf.init(t, x0, P0);

      //Perform one round of the Kalman filter
      y << xcoord, ycoord;
      kf.predict();
      kf.update(y);

      //Create the trajectory for the current dynamic obstacle
      Eigen::MatrixXd safety_margin(n, n);
      
      for (int i = 0; i < params_.KFiterations; i++)
      {
        kf.predict();
        xcoords.push_back(kf.state()(0));
        ycoords.push_back(kf.state()(1));
        zcoords.push_back(zcoord);
        if(i == safety_iteration)
        {
          //Because of controllability we can only extract one of these
          //ellipses. See paper for explanation.
          safety_margin = kf.covariance();
        }
      }

      std::fill(covariance_matrices.begin(), covariance_matrices.end(), safety_margin);  
      kf_data.push_back(std::make_pair(std::make_tuple(xcoords, ycoords, zcoords), covariance_matrices)); 
  }
    return kf_data;
}

/*
* Gets the correct covariance ellipse from a prediction given time t.
*/
int AEPlanner::getCovarianceIndex(double max_time_step, double time_step, double t){
  
  int num_steps = static_cast<int>(max_time_step / time_step); // Calculate the number of time steps

  if(t > max_time_step)
  {
    //No prediction available
    return -1;
  }

  // Initialize the minimum difference and index
  double min_difference = std::abs(t - time_step);
  int index = 0;

  // Loop through each time step and find the closest one
  for (int i = 1; i < num_steps; ++i) {
    double time = i * time_step; // Calculate the time for the current step
    double difference = std::abs(t - time);
    if (difference < min_difference) {
      min_difference = difference;
      index = i;
    }
  }
  return index;
}

/*
* Check intersection between a circle around a goal and the predicted covariance ellipse (currently implemented for a circle).
*/
bool AEPlanner::isCircleInsideEllipse(const Eigen::Vector3d& point, const Eigen::Vector3d& center, 
            std::tuple<double, double, Eigen::MatrixXd> covariance_ellipse) 
{ 
  /*
  // Radius around goal
  double radius = 0.2;

  // Extract major and minor axis lengths and eigenvectors
  double major_axis_length = std::get<0>(covariance_ellipse);
  double minor_axis_length = std::get<1>(covariance_ellipse);
  Eigen::MatrixXd eigenvectors = std::get<2>(covariance_ellipse);
  
  // Calculate relative coordinates of the point with respect to the ellipse center (XY)
  Eigen::Vector2d relative_point = point.head(2) - center.head(2);

  // Rotate the relative coordinates by the angle of the major axis
  Eigen::Vector2d rotated_relative_point = eigenvectors.transpose() * relative_point;

  // Normalize the rotated relative coordinates
  double normalized_x = rotated_relative_point(0) / major_axis_length;
  double normalized_y = rotated_relative_point(1) / minor_axis_length;

  // Check if the normalized coordinates are inside the unit circle
  double distance_from_origin = normalized_x * normalized_x + normalized_y * normalized_y;
  
  // Calculate the normalized radius of the circle
  double normalized_radius = radius / std::max(major_axis_length, minor_axis_length);

  // Check if the circle intersects with the ellipse
  bool does_intersect_XY = distance_from_origin <= 1.0 + normalized_radius;
  // Check if the sphere intersects with the cylinder
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + params_.human_height + (2*params_.drone_height);
  return does_intersect_XYZ;
  */
  double circle_radius = std::get<0>(covariance_ellipse);
  // Calculate the distance between the centers of the two circles
  Eigen::Vector2d point_XY = point.head(2);
  double distance = (point_XY - center.head(2)).norm();

  // Check if the two circles intersect
  bool does_intersect_XY = distance <= 0.2 + circle_radius;
  bool does_intersect_XYZ = does_intersect_XY and point(2) < center(2) + params_.human_height + (2*params_.drone_height);

  return does_intersect_XYZ;

}


/* 
* Calculate if there is a potential collision with any moving obstacle in time t.
*/
bool AEPlanner::checkCollision(double t, 
                              Eigen::Vector4d point, 
                              std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                              std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  double max_time_step = params_.KFiterations * params_.time_step;
  int N_persons = trajectories.size();
  
  // Index the correct circle
  int covariance_index = getCovarianceIndex(max_time_step, params_.time_step, t);

  if(covariance_index == -1)
  { 
    // Outside our prediction
    return false;
  }

  for(int i = 0; i < N_persons; i++)
  { 
      
      auto personEllipses = covarianceEllipses[i];
      auto personTrajectory = trajectories[i];
      
      //Extract the correct mean and ellipse
      std::tuple<double, double, Eigen::MatrixXd> covarianceEllipse = personEllipses[covariance_index];
      
      double ellipse_center_x = std::get<0>(personTrajectory)[covariance_index];
      double ellipse_center_y = std::get<1>(personTrajectory)[covariance_index];
      double ellipse_center_z = std::get<2>(personTrajectory)[covariance_index];

      double person_current_x = std::get<0>(personTrajectory)[0];
      double person_current_y = std::get<1>(personTrajectory)[0];

      //if(sqrt(pow(person_current_x-point[0], 2.0) + pow(person_current_y-point[1], 2.0)) < params_.KFiterations * params_.time_step * params_.human_linear_velocity)
      //{
        // Check if person is close enough
        const Eigen::Vector3d center(ellipse_center_x, ellipse_center_y, ellipse_center_z);
        
        if(isCircleInsideEllipse(point.head(3), center, covarianceEllipse)) {
          return true;
        }
      //}
  }
  return false;
}


void AEPlanner::execute(const aeplanner::aeplannerGoalConstPtr& goal)
{
  aeplanner::aeplannerResult result;

  // Check if aeplanner has recieved agent's pose yet
  if (!current_state_initialized_)
  {
    ROS_WARN("Agent's pose not yet received");
    ROS_WARN("Make sure it is being published and correctly mapped");
    as_.setSucceeded(result);
    return;
  }
  if (!ot_)
  {
    ROS_WARN("No octomap received");
    as_.setSucceeded(result);
    return;
  }

  // Perform prediction of human obstacles future positions 
  // using the Kalman Filter with Constant Velocity Model
  auto new_vec = KFpredictTrajectories();
  { 
    // Race condition
    std::lock_guard<std::mutex> lock(vecMutex);
    predicted_data = new_vec;
  }

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories{};
  std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> all_ellipses{};

  // Separate the covariance ellipses and the trajectories.
  for(auto const& person : predicted_data)
  {
    trajectories.push_back(person.first);
    all_ellipses.push_back(createCovarianceEllipse(person.second));
  }

  // Visualize covariance and predicted trajectory
  visualizePrediction(pred_marker_pub_, covariance_marker_pub_, trajectories, all_ellipses);

  RRTNode* root = initialize(trajectories, all_ellipses);

  // Check if we have a old best branch
  if (best_branch_root_)
  {
    // Check if previous branch is collision free
    std::pair<RRTNode*, bool> result = pathIsSafe(best_branch_root_, trajectories, all_ellipses);
    if(result.second)
    {
      // Get candidate goal
      expandRRT(trajectories, all_ellipses);

      // Check if we got a valid candidate node
      if(best_node_)
      {
        // Compare candidate goal with previous goal
        double old_score = result.first->dynamic_score(params_.lambda, params_.zeta);
        double new_score = best_node_->dynamic_score(params_.lambda, params_.zeta);
        
        // if new goal is less than 10% better, choose old goal
        if(new_score/old_score < 1.1){
          best_node_ = result.first;
        }
      }
    }
    else 
    {
      // Old branch contains collision
      expandRRT(trajectories, all_ellipses);
    }
  }
  else 
  {
    //No best branch available
    expandRRT(trajectories, all_ellipses);
  }

  // No valid nodes found in expandRRT
  if (best_node_ == NULL){
    // we cant find a valid node, --> frontier planning
    ROS_DEBUG_STREAM("GLOBAL PLANNING");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    as_.setSucceeded(result);
    ROS_DEBUG("Deleting/Freeing!");
    delete root;
    kd_free(kd_tree_);
    ROS_DEBUG("Done!");
    return;
  }

  ROS_DEBUG_STREAM("getCopyOfParent");
  best_branch_root_ = best_node_->getCopyOfParentBranch();

  ROS_DEBUG_STREAM("createRRTMarker");
  rrt_marker_pub_.publish(createRRTMarkerArray(root, params_.lambda));
  ROS_DEBUG_STREAM("publishRecursive");
  publishEvaluatedNodesRecursive(root);

  // Only one action along best branch is executed.
  // Defensive check: some edge cases can leave the cached branch without a next node.
  if (!best_branch_root_ || best_branch_root_->children_.empty())
  {
    ROS_WARN("Best branch has no executable child node. Falling back to frontier planning.");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    as_.setSucceeded(result);
    delete root;
    kd_free(kd_tree_);
    return;
  }
  result.pose.pose = vecToPose(best_branch_root_->children_[0]->state_);

  // If we find a best node
  if (best_node_->dynamic_score(params_.lambda, params_.zeta) > params_.zero_gain)
  {
    result.is_clear = true;
    logTreeGuidanceDecision(
        goal->header.seq,
        goal->actions_taken,
        best_branch_root_->children_[0],
        best_node_,
        "aep");
    ROS_DEBUG_STREAM("LOCAL PLANNING");
  }
  else
  {
    // we cant find a valid node, --> frontier planning
    ROS_DEBUG_STREAM("GLOBAL PLANNING");
    result.frontiers = getFrontiers();
    result.is_clear = false;
    delete best_branch_root_;
    best_branch_root_ = NULL;
  }

  as_.setSucceeded(result);
  ROS_DEBUG_STREAM("Deleting/Freeing!");
  delete root;
  kd_free(kd_tree_);
  ROS_DEBUG_STREAM("Done!");
}


RRTNode* AEPlanner::initialize(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                               std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  // Initialize kd-tree
  /* create a kd-tree for 3-dimensional data */
  kd_tree_ = kd_create(3);
  best_node_ = NULL;
  RRTNode* root = NULL;
  if (best_branch_root_)
  {
    reevaluatePotentialInformationGainRecursive(best_branch_root_);

    RRTNode* old_root = best_branch_root_;
    if (old_root->children_.empty())
    {
      delete old_root;
      best_branch_root_ = NULL;
    }
    else
    {
      best_branch_root_ = old_root->children_[0];
      old_root->children_.clear();
      delete old_root;

      VisualizeOldPath(old_path_marker_pub_, best_branch_root_);
      ROS_DEBUG_STREAM("best branch root : (" << best_branch_root_->state_(0) << ", " << best_branch_root_->state_(1) << ", " << best_branch_root_->state_(2) << ", " << best_branch_root_->state_(3) << ")");

      if (best_branch_root_->children_.size() > 0)
      {
        best_branch_root_->parent_ = NULL;
      }
      else
      {
        delete best_branch_root_;
        best_branch_root_ = NULL;
      }
    }
  }
  

  // Initialize without any previous branch
  root = new RRTNode();
  root->state_[0] = current_state_[0];
  root->state_[1] = current_state_[1];
  root->state_[2] = current_state_[2];
  ROS_DEBUG_STREAM("kd_insert3 x: " << current_state_[0] << " y: "<< current_state_[1] << " z: "<< current_state_[2] << " yaw: "<< current_state_[3]);
  kd_insert3(kd_tree_, root->state_[0], root->state_[1], root->state_[2], root);
  return root;
}


/*
Check if a branch is predicted to be collision free
*/
std::pair<RRTNode*, bool> AEPlanner::pathIsSafe(RRTNode* node, 
                                                std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                                                std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  if(node == NULL){
    return std::make_pair(nullptr, false);
  }

  RRTNode* curr = node;
  // Traverse branch node-by-node, including the leaf.
  while (curr) {
    double time_to_reach_node = curr->time_cost();

    bool collision = checkCollision(time_to_reach_node, curr->state_, trajectories, covarianceEllipses) && dynamic_mode_;
    if (collision) {
      return std::make_pair(nullptr, false);  // Collision detected, return false
    }

    if (curr->children_.empty()) {
      return std::make_pair(curr, true);  // No collision, leaf is safe
    }

    curr = curr->children_[0];
  }  

  return std::make_pair(nullptr, false);
}

void AEPlanner::reevaluatePotentialInformationGainRecursive(RRTNode* node)
{
 std::tuple<double, double, double> ret = getGain(node, node->time_cost());
  node->gain_ = std::get<0>(ret); // Assign static gain
  node->dynamic_gain_ = std::get<1>(ret); //Assign dynamic gain
  node->state_[3] = std::get<2>(ret); // Assign yaw angle that maximizes static gain

  //Acquire DFM score
  pigain::QueryDFMRequest req;
  req.point.x = node->state_[0];
  req.point.y = node->state_[1];
  req.point.z =  0;
  pigain::QueryDFMResponse res;
  if(dfm_client_.call(req, res))
  {
    node->dfm_score_ = res.score; 
  }


  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
    reevaluatePotentialInformationGainRecursive(*node_it);
}


void AEPlanner::expandRRT(std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> trajectories, 
                          std::vector<std::vector<std::tuple<double, double, Eigen::MatrixXd>>> covarianceEllipses)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double estimated_yaw = 0.0;

  // Expand an RRT tree and calculate information gain in every node
  ROS_DEBUG_STREAM("Entering expanding RRT");
  std::vector<Eigen::Vector4d> validNodes{};
  std::vector<Eigen::Vector4d> candidateNodes{};
  int N_valid = 0; // Valid sampled nodes counter
  int N_sampled_nodes = 0; 

  // (1) Sample until N_valid > init_iterations and total sampled nodes < max_sampled_nodes
  // (2) If more than max_sampled_nodes are sampled: continue until max_sampled_nodes as long as dynamic score is not high enough
  while ((N_valid < params_.init_iterations and N_sampled_nodes < params_.max_sampled_initial_nodes) or 
  ((N_valid > 0 and N_sampled_nodes < params_.max_sampled_nodes and best_node_ and best_node_->dynamic_score(params_.lambda, params_.zeta) < params_.zero_gain) 
  and ros::ok()))

  {
    RRTNode* new_node = new RRTNode();
    RRTNode* nearest;
    octomap::OcTreeNode* ot_result;
    
    // Sample new point around agent and check that
    // (1) it is within the boundaries
    // (2) it is in known space
    // (3) the path between the new node and it's parent does not contain any
    // obstacles

    do
    {
      Eigen::Vector4d offset = sampleNewPoint();
      new_node->state_ = current_state_ + offset;
      ROS_DEBUG_STREAM("sample x: " << new_node->state_[0] << " y: "<< new_node->state_[1] << " z: "<< new_node->state_[2]);

      nearest = chooseParent(new_node, params_.extension_range);
      if (!nearest)
      {
        continue;
      }

      new_node->state_ = restrictDistance(nearest->state_, new_node->state_);
      ROS_DEBUG_STREAM("restrictDistance sample x: " << new_node->state_[0] << " y: "<< new_node->state_[1] << " z: "<< new_node->state_[2]);
      
      ROS_DEBUG_STREAM("Trying node (" << new_node->state_[0] << ", "
                                       << new_node->state_[1] << ", "
                                       << new_node->state_[2] << ")");
      ROS_DEBUG_STREAM("nearest (" << nearest->state_[0] << ", " << nearest->state_[1]
                                       << ", " << nearest->state_[2] << ")");
      ot_result = ot->search(octomap::point3d(new_node->state_[0], new_node->state_[1],
                                              new_node->state_[2]));
      if (ot_result == NULL)
        continue;
      
      ROS_DEBUG_STREAM("ot check done!");
      
      ROS_DEBUG_STREAM("Inside boundaries?  " << isInsideBoundaries(new_node->state_));
      ROS_DEBUG_STREAM("In known space?     " << ot_result);
      ROS_DEBUG_STREAM("Collision?          " << collisionLine(nearest->state_, new_node->state_, params_.bounding_radius));

    } while (!isInsideBoundaries(new_node->state_) or 
             !ot_result or
             collisionLine(nearest->state_, new_node->state_, params_.bounding_radius));


    pigain::Query srv;
    srv.request.point.x = new_node->state_[0];
    srv.request.point.y = new_node->state_[1];
    srv.request.point.z = new_node->state_[2];

    // Estimate the yaw using nearest neighbor
    if (nn_yaw_query_client_.call(srv))
    {
        estimated_yaw = srv.response.yaw;
    }
    new_node->state_[3] = estimated_yaw;

    // Estimate time to reach node
    double time_to_reach_node = nearest->time_cost() + nearest->time_to_reach(new_node);

    bool collision = checkCollision(time_to_reach_node, new_node->state_, trajectories, covarianceEllipses) && dynamic_mode_;
    if (!collision) 
    {
      ROS_DEBUG_STREAM("Get gain");
      std::tuple<double, double, double> ret = getGain(new_node, time_to_reach_node);
      // Update node
      new_node->gain_ = std::get<0>(ret);
      new_node->dynamic_gain_ = std::get<1>(ret);
      new_node->state_[3] = std::get<2>(ret); //Set new yaw that maximizes gain

      //Aquire DFM score
      pigain::QueryDFMRequest req;
      req.point.x = new_node->state_[0];
      req.point.y = new_node->state_[1];
      req.point.z =  0;
      pigain::QueryDFMResponse res;
      if(dfm_client_.call(req, res))
      {
        new_node->dfm_score_ = res.score; 
      }

      N_valid++;
      // new_node is now ready to be added to tree
      new_node->parent_ = nearest; 
      
      nearest->children_.push_back(new_node);

      // rewire tree with new node
      ROS_DEBUG_STREAM("rewire start");
      rewire(kd_tree_, nearest, params_.extension_range, params_.bounding_radius,
            params_.d_overshoot_);

      ROS_DEBUG_STREAM("Insert into KDTREE");
      kd_insert3(kd_tree_, new_node->state_[0], new_node->state_[1], new_node->state_[2],
                new_node);

      // Update best node
      ROS_DEBUG_STREAM("Update best node");
      if (!best_node_ or
          new_node->dynamic_score(params_.lambda, params_.zeta) > best_node_->dynamic_score(params_.lambda, params_.zeta))
        best_node_ = new_node;
      

      // Estimate time to reach node (Now we know the yaw)
      Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
      validNodes.push_back(p4);
      ROS_DEBUG_STREAM("iteration Done!");
    } 
    else 
    {
      // Estimate time to reach node (Now we know the yaw)
      Eigen::Vector4d p4(new_node->state_[0], new_node->state_[1], new_node->state_[2], int(collision));
      validNodes.push_back(p4);
      ROS_DEBUG_STREAM("Collision, node not valid !");
    }
    N_sampled_nodes++;
  }

  if (N_valid > 0 && best_node_) {
  // Visualize the best node as green (=2)
  Eigen::Vector4d p4(best_node_->state_[0], best_node_->state_[1], best_node_->state_[2], 2);
  validNodes.push_back(p4);
  visualizeBestNode(best_marker_pub_, validNodes);
  }
  else
  {
    ROS_DEBUG_STREAM("No valid nodes found!");
  }
  ROS_DEBUG_STREAM("expandRRT Done!");
}

Eigen::Vector4d AEPlanner::sampleNewPoint()
{
  // Samples one point uniformly over a sphere with a radius of
  // param_.max_sampling_radius
  Eigen::Vector4d point;
  do
  {
    for (int i = 0; i < 3; i++)
      point[i] = params_.max_sampling_radius * 2.0 *
                 (((double)rand()) / ((double)RAND_MAX) - 0.5);
  } while (pow(point[0], 2.0) + pow(point[1], 2.0) + pow(point[2], 2.0) >
           pow(params_.max_sampling_radius, 2.0));

  return point;
}

RRTNode* AEPlanner::chooseParent(RRTNode* node, double l)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  // Find nearest neighbour
  kdres* nearest = kd_nearest_range3(kd_tree_, node->state_[0], node->state_[1],
                                     node->state_[2], l + 0.5); // FIXME why +0.5?

  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    nearest = kd_nearest3(kd_tree_, node->state_[0], node->state_[1], node->state_[2]);
  }
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return NULL;
  }

  RRTNode* node_nn = (RRTNode*)kd_res_item_data(nearest);

  RRTNode* best_node = node_nn;
  double best_node_cost = best_node->cost();
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    double node_cost = node_nn->cost();
    if (best_node and node_cost < best_node_cost)
    {
      best_node = node_nn;
      best_node_cost = node_cost;
    }

    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

void AEPlanner::rewire(kdtree* kd_tree, RRTNode* new_node, double l, double r,
                       double r_os)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
  Eigen::Vector4d current_state = current_state_;

  RRTNode* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->state_[0], new_node->state_[1],
                                     new_node->state_[2], l + 0.5); // FIXME why +0.5?
  while (!kd_res_end(nearest))
  {
    node_nn = (RRTNode*)kd_res_item_data(nearest);
    Eigen::Vector3d p1(new_node->state_[0], new_node->state_[1], new_node->state_[2]);
    Eigen::Vector3d p2(node_nn->state_[0], node_nn->state_[1], node_nn->state_[2]);
    if (node_nn->cost() > new_node->cost() + (p1 - p2).norm())
    {
      if (!collisionLine(new_node->state_, node_nn->state_, r))
        node_nn->parent_ = new_node;
    }
    kd_res_next(nearest);
  }
  kd_res_free(nearest);
}

Eigen::Vector4d AEPlanner::restrictDistance(Eigen::Vector4d nearest,
                                            Eigen::Vector4d new_pos)
{
  // Check for collision
  Eigen::Vector3d origin(nearest[0], nearest[1], nearest[2]);
  Eigen::Vector3d direction(new_pos[0] - origin[0], new_pos[1] - origin[1],
                            new_pos[2] - origin[2]);
  if (direction.norm() > params_.extension_range)
    direction = params_.extension_range * direction.normalized();

  new_pos[0] = origin[0] + direction[0];
  new_pos[1] = origin[1] + direction[1];
  new_pos[2] = origin[2] + direction[2];

  return new_pos;
}

std::tuple<double, double, double> AEPlanner::getGain(RRTNode* node, double time_of_arrival)
{
  pigain::Query srv;
  srv.request.point.x = node->state_[0];
  srv.request.point.y = node->state_[1];
  srv.request.point.z = node->state_[2];
  
  // GP currently deactivated
  //if (gp_query_client_.call(srv))
  //{
  //  double gain = srv.response.mu;
  //  double yaw = srv.response.yaw;
  //  ROS_DEBUG_STREAM("gain impl: " << gain);
  //  ROS_DEBUG_STREAM("sigma: " << srv.response.sigma);
   // if (srv.response.sigma < params_.sigma_thresh)
   // {
   //   double gain = srv.response.mu;
   //   double yaw = srv.response.yaw;
   //
   //   ROS_DEBUG_STREAM("gain impl: " << gain);
   //   return std::make_pair(gain, yaw);
   // }
  //}
  
  node->gain_explicitly_calculated_ = true;  
  std::tuple<double, double, double> ret = gainCubature(node->state_, time_of_arrival);
  applyTreeGuidanceToNode(node, time_of_arrival, ret);
  ROS_DEBUG_STREAM("gain expl: " << std::get<0>(ret));
  return ret;
}

bool AEPlanner::reevaluate(aeplanner::Reevaluate::Request& req,
                           aeplanner::Reevaluate::Response& res)
{

  for (std::vector<geometry_msgs::Point>::iterator it = req.points.begin();
       it != req.points.end(); ++it)
  {
    Eigen::Vector4d pos(it->x, it->y, it->z, 0);

    //Compute the current gain in each cached node
    std::tuple <double, double, double> gain_response = gainCubature(pos, 0);
    if (params_.tree_guidance_enabled)
    {
      const TreeGainBreakdown tree_gain = treeInformationGainBreakdown(pos, 0.0);
      std::get<1>(gain_response) += params_.tree_gain_weight * tree_gain.score;
    }
    ROS_DEBUG_STREAM("gain reeval expl: " << std::get<0>(gain_response));

    //Add the result for each node in a response list
    res.gain.push_back(std::get<0>(gain_response));
    res.dynamic_gain.push_back(std::get<1>(gain_response));
    res.yaw.push_back(std::get<2>(gain_response));
  }
  return true;
}

/**
 * This function calculates the gain explicitly from the given state (point).
 * It calculates the dynamic gain as well as the static gain and returns them in a tuple
 * with the best yaw angle.
 */
std::tuple <double, double, double> AEPlanner::gainCubature(Eigen::Vector4d state, double time_of_arrival)
{
  visualization_msgs::MarkerArray static_rays;
  visualization_msgs::MarkerArray dynamic_rays;

  // **Declare intenal variables**
  std::shared_ptr<octomap::OcTree> ot = ot_;
  double static_gain = 0.0;
  double dynamic_gain = 0.0;
  if (!ot)
  {
    ROS_WARN_THROTTLE(2.0, "gainCubature called without octomap. Returning zero gain.");
    return std::make_tuple(0.0, 0.0, state[3]);
  }

  //Field of View
  double fov_y = params_.hfov, fov_p = params_.vfov; //Horizontoal 103.2 deg and Vertical 77.4 deg

  //Radius variables
  double r;
  double r_min = params_.r_min, r_max = params_.r_max;

  //Angles
  int phi, theta;
  double phi_rad, theta_rad;
  
  //Step size for r, phi and theta. In degrees and radians.
  double dr = params_.dr, dphi = params_.dphi, dtheta = params_.dtheta;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;

  //Maps
  std::map<int, double> gain_per_yaw;
  std::map<int, double> dynamic_gain_per_yaw;

  //Set the origin to the point to be examined
  Eigen::Vector3d origin(state[0], state[1], state[2]);
  Eigen::Vector3d vec;

  double max_time_step = params_.KFiterations * params_.time_step;

  int index = getCovarianceIndex(max_time_step, params_.time_step, time_of_arrival);
  int ray_id = 0;
  bool blocked = false;

  //For each theta in a circle
  for (theta = -180; theta < 180; theta += dtheta)
  {
    theta_rad = M_PI * theta / 180.0f;
    //For each phi in pitch
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi)
    {
      phi_rad = M_PI * phi / 180.0f;

      //Gain for a specific theta and phi over all r
      double static_g = 0.0;
      double dynamic_g = 0.0;
      visualization_msgs::Marker static_ray;
      visualization_msgs::Marker dynamic_ray;
      blocked = false;

      if(params_.visualize_static_and_dynamic_rays)
      {
        static_ray = createRayMarker(ray_id, "BLUE");
        dynamic_ray = createRayMarker(ray_id, "RED");
      }
      
      //For each r in distance from origin to r_max
      for (r = r_min; r < r_max; r += dr)
      {
        //A point in x,y,z from spherical coordinates
        //So from this volume we are looping over, we extract one point
        vec[0] = origin[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = origin[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = origin[2] + r * cos(phi_rad);

        //Search for the x,y,z point in the OctomapTree
        octomap::point3d query(vec[0], vec[1], vec[2]);
        octomap::OcTreeNode* result = ot->search(query);

        //Construct a Vector4d from the point but with yaw = 0
        Eigen::Vector4d v(vec[0], vec[1], vec[2], 0);

        geometry_msgs::Point point;
        point.x = vec[0];
        point.y = vec[1];
        point.z = vec[2];

        if (not blocked and dynamic_mode_ and index != -1)
        {
          blocked = willViewBeBlocked(vec, index, params_.visualize_static_and_dynamic_rays);

        }

        //If the point is outside the map, stop computing gain in that direction
        if (!isInsideBoundaries(v))
        {
          break;
        }

        if (result) //If the point is in known space
        {
          //Iterating forward with r, if the current point is occupied it means 
          //that we have hit a static obstacle
          if (result->getLogOdds() > 0)
          {
            //So break the r loop
            break;
          }

          //If the point was free, do not break the loop so continue to loop over r
          //since there might be more unknown space to be found after the known space.
        }
        else 
        { 
          double current_gain = (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2); 
          if(pointOnXYBoundaries(v)){
            current_gain *= params_.boost_magnitude;
          }
               
          if(blocked)
          {
            //The view is blocked by a dynamic obstacle
            //Stop adding gain to the dynamic gain, but continue to add to the static one
            static_g += current_gain;
          }
          else
          {
            //The view is not blocked, add the same gain to static and dynamic
            static_g += current_gain;
            dynamic_g += current_gain;
          }
        }

        if(params_.visualize_static_and_dynamic_rays)
        {
          if(blocked)
          {
            //Visualize static rays behind obstacle
            static_ray.points.push_back(point);
          }
          else
          {
            //Visualize both dynamic and static rays
            static_ray.points.push_back(point);
            dynamic_ray.points.push_back(point);
          }
        }

      }
      gain_per_yaw[theta] += static_g; //Add for each theta what the current gain is (Yaw)
      dynamic_gain_per_yaw[theta] += dynamic_g; //Add for each theta what the current dymamic_gain is.
      
      if(params_.visualize_static_and_dynamic_rays)
      {
        static_rays.markers.push_back(static_ray); //Add ray to rays for visualize
        if(blocked)
        {
          dynamic_rays.markers.push_back(dynamic_ray); //Add ray to rays for visualize  
        }
        else
        {
          dynamic_ray = createRayMarker(ray_id, "RED");
          dynamic_rays.markers.push_back(dynamic_ray);
        }
        ray_id++;
      }
    }
  }

  if(params_.visualize_static_and_dynamic_rays)
  {
    static_rays_marker_pub_.publish(static_rays);
    dynamic_rays_marker_pub_.publish(dynamic_rays);
  }

  //Calculate the best yaw angle
  int best_yaw = 0;
  double best_yaw_score = 0.0, best_dynamic_yaw_score = 0.0;
  
  //For each yaw in one spin
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    double dynamic_yaw_score = 0;
    //For each field of view in the y-direction
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
      dynamic_yaw_score += dynamic_gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_dynamic_yaw_score = dynamic_yaw_score;
      best_yaw = yaw;
    }
  }

  static_gain = best_yaw_score;
  dynamic_gain = best_dynamic_yaw_score;
  double yaw = M_PI * best_yaw / 180.f; //deg2rad
  state[3] = yaw;

  return std::make_tuple(static_gain, dynamic_gain, yaw);
}

/**
 * Check if a point will intersect with a dynamic obstacle at time_of_arrival
 * if that is the case, the view will be blocked and we return true. 
*/
bool AEPlanner::willViewBeBlocked(Eigen::Vector3d point, int index, bool visualize_ghosts)
{
  // Race condition
  std::lock_guard<std::mutex> lock(vecMutex);

  if(visualize_ghosts)
  {
    std::vector<std::tuple<double, double, double>> positions;
    for(auto const& person : predicted_data)
    { 
      double x = std::get<0>(person.first)[index];
      double y = std::get<1>(person.first)[index];
      double z = std::get<2>(person.first)[index];
      std::tuple<double, double, double> position = std::make_tuple(x,y,z);
      positions.push_back(position);
    }
    visualizeGhostPedestrian(ghost_marker_pub_, positions);
  }
  
  for(auto const& person : predicted_data)
  { 
    double x = std::get<0>(person.first)[index];
    double y = std::get<1>(person.first)[index];
    double z = std::get<2>(person.first)[index];
    if(isCollisionWithBoundingBox(point, x, y, z))
    {
      return true;
    }
    
  }
  return false;
}

/**
 * Check if a certain point collides with a bounding box, in this case the square
 * of a human obstacle.
*/
bool AEPlanner::isCollisionWithBoundingBox(Eigen::Vector3d point, double x, double y, double z)
{
  return point[0] >= (x - params_.human_width/2) and point[0] <= (x + params_.human_width/2) and
         point[1] >= (y - params_.human_width/2) and point[1] <= (y + params_.human_width/2) and
         point[2] >= z and point[2] <= (z + params_.human_height);
}


geometry_msgs::PoseArray AEPlanner::getFrontiers()
{
  geometry_msgs::PoseArray frontiers;

  pigain::BestNode srv;
  srv.request.threshold = params_.cache_node_threshold; 
  if (best_node_client_.call(srv))
  {
    for (int i = 0; i < srv.response.best_node.size(); ++i)
    {
      geometry_msgs::Pose frontier;
      frontier.position = srv.response.best_node[i];
      frontiers.poses.push_back(frontier);
    }
  }
  return frontiers;
}


bool AEPlanner::isInsideBoundaries(Eigen::Vector4d point)
{  
  return point[0] > params_.boundary_min[0] and point[0] < params_.boundary_max[0] and
         point[1] > params_.boundary_min[1] and point[1] < params_.boundary_max[1] and
         point[2] > params_.boundary_min[2] and point[2] < params_.boundary_max[2];
}

bool AEPlanner::pointOnXYBoundaries(Eigen::Vector4d point)
{ 
  if(point[0] < params_.boundary_min[0] + params_.boosted_boundary_length){
    return true;
  }
  if(point[0] > params_.boundary_max[0] - params_.boosted_boundary_length){
    return true;
  }
  if(point[1] < params_.boundary_min[1] + params_.boosted_boundary_length){
    return true;
  }
  if(point[1] > params_.boundary_max[1] - params_.boosted_boundary_length){
    return true;
  }
  return false;
}


bool AEPlanner::collisionLine(Eigen::Vector4d p1, Eigen::Vector4d p2, double r)
{
  std::shared_ptr<octomap::OcTree> ot = ot_;
  ROS_DEBUG_STREAM("In collision");
  octomap::point3d start(p1[0], p1[1], p1[2]);
  octomap::point3d end(p2[0], p2[1], p2[2]);
  octomap::point3d min(std::min(p1[0], p2[0]) - r, std::min(p1[1], p2[1]) - r,
                       std::min(p1[2], p2[2]) - r);
  octomap::point3d max(std::max(p1[0], p2[0]) + r, std::max(p1[1], p2[1]) + r,
                       std::max(p1[2], p2[2]) + r);
  double lsq = (end - start).norm_sq();
  double rsq = r * r;

  for (octomap::OcTree::leaf_bbx_iterator it = ot->begin_leafs_bbx(min, max),
                                          it_end = ot->end_leafs_bbx();
       it != it_end; ++it)
  {
    octomap::point3d pt(it.getX(), it.getY(), it.getZ());

    if (it->getLogOdds() > 0)  // Node is occupied
    {
      if (CylTest_CapsFirst(start, end, lsq, rsq, pt) > 0 or (end - pt).norm() < r)
      {
        return true;
      }
    }
  }
  ROS_DEBUG_STREAM("In collision (exiting)");

  return false;
}


void AEPlanner::octomapCallback(const octomap_msgs::Octomap& msg)
{
  ROS_DEBUG_STREAM("Freeing ot_");
  octomap::AbstractOcTree* aot = octomap_msgs::msgToMap(msg);
  octomap::OcTree* ot = (octomap::OcTree*)aot;
  ot_ = std::make_shared<octomap::OcTree>(*ot);

  delete ot;
  ROS_DEBUG_STREAM("Freeing ot_ done:");
}

void AEPlanner::publishEvaluatedNodesRecursive(RRTNode* node)
{
  if (!node)
    return;
  for (typename std::vector<RRTNode*>::iterator node_it = node->children_.begin();
       node_it != node->children_.end(); ++node_it)
  {
    if ((*node_it)->gain_explicitly_calculated_)
    {
      pigain::Node pig_node;
      pig_node.gain = (*node_it)->gain_;
      pig_node.dynamic_gain = (*node_it)->dynamic_gain_;
      pig_node.position.x = (*node_it)->state_[0];
      pig_node.position.y = (*node_it)->state_[1];
      pig_node.position.z = (*node_it)->state_[2];
      pig_node.yaw = (*node_it)->state_[3];
      gain_pub_.publish(pig_node);
    }

    publishEvaluatedNodesRecursive(*node_it);
  }
}


void AEPlanner::agentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_state_[0] = msg.pose.position.x;
  current_state_[1] = msg.pose.position.y;
  current_state_[2] = msg.pose.position.z;
  current_state_[3] = tf2::getYaw(msg.pose.orientation);
  current_state_initialized_ = true;
}


geometry_msgs::Pose AEPlanner::vecToPose(Eigen::Vector4d state)
{
  tf::Vector3 origin(state[0], state[1], state[2]);
  double yaw = state[3];

  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, yaw);
  tf::Pose pose_tf(quat, origin);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(pose_tf, pose);

  return pose;
}

//-----------------------------------------------------------------------------
// Name: CylTest_CapsFirst
// Orig: Greg James - gjames@NVIDIA.com
// Lisc: Free code - no warranty & no money back.  Use it all you want
// Desc:
//    This function tests if the 3D point 'pt' lies within an arbitrarily
// oriented cylinder.  The cylinder is defined by an axis from 'pt1' to 'pt2',
// the axis having a length squared of 'lsq' (pre-compute for each cylinder
// to avoid repeated work!), and radius squared of 'rsq'.
//    The function tests against the end caps first, which is cheap -> only
// a single dot product to test against the parallel cylinder caps.  If the
// point is within these, more work is done to find the distance of the point
// from the cylinder axis.
//    Fancy Math (TM) makes the whole test possible with only two dot-products
// a subtract, and two multiplies.  For clarity, the 2nd mult is kept as a
// divide.  It might be faster to change this to a mult by also passing in
// 1/lengthsq and using that instead.
//    Elminiate the first 3 subtracts by specifying the cylinder as a base
// point on one end cap and a vector to the other end cap (pass in {dx,dy,dz}
// instead of 'pt2' ).
//
//    The dot product is constant along a plane perpendicular to a vector.
//    The magnitude of the cross product divided by one vector length is
// constant along a cylinder surface defined by the other vector as axis.
//
// Return:  -1.0 if point is outside the cylinder
// Return:  distance squared from cylinder axis if point is inside.
//
//-----------------------------------------------------------------------------
float AEPlanner::CylTest_CapsFirst(const octomap::point3d& pt1,
                                   const octomap::point3d& pt2, float lsq, float rsq,
                                   const octomap::point3d& pt)
{
  float dx, dy, dz;     // vector d  from line segment point 1 to point 2
  float pdx, pdy, pdz;  // vector pd from point 1 to test point
  float dot, dsq;

  dx = pt2.x() - pt1.x();  // translate so pt1 is origin.  Make vector from
  dy = pt2.y() - pt1.y();  // pt1 to pt2.  Need for this is easily eliminated
  dz = pt2.z() - pt1.z();

  pdx = pt.x() - pt1.x();  // vector from pt1 to test point.
  pdy = pt.y() - pt1.y();
  pdz = pt.z() - pt1.z();

  // Dot the d and pd vectors to see if point lies behind the
  // cylinder cap at pt1.x, pt1.y, pt1.z

  dot = pdx * dx + pdy * dy + pdz * dz;

  // If dot is less than zero the point is behind the pt1 cap.
  // If greater than the cylinder axis line segment length squared
  // then the point is outside the other end cap at pt2.

  if (dot < 0.0f || dot > lsq)
    return (-1.0f);
  else
  {
    // Point lies within the parallel caps, so find
    // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
    // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
    // Carefull: '*' means mult for scalars and dotproduct for vectors
    // In short, where dist is pt distance to cyl axis:
    // dist = sin( pd to d ) * |pd|
    // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
    // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
    // dsq = pd * pd - dot * dot / lengthsq
    //  where lengthsq is d*d or |d|^2 that is passed into this function

    // distance squared to the cylinder axis:

    dsq = (pdx * pdx + pdy * pdy + pdz * pdz) - dot * dot / lsq;

    if (dsq > rsq)
      return (-1.0f);
    else
      return (dsq);  // return distance squared to axis
  }
}

}  // namespace aeplanner
