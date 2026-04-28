#ifndef READ_PARAMS_H
#define READ_PARAMS_H

#include <string>
#include <vector>

namespace aeplanner
{
  struct Params
  {
    double hfov;
    double vfov;
    double r_max;
    double r_min;

    double dr;
    double dphi;
    double dtheta;

    double lambda;
    double zero_gain;
    double extension_range;
    double max_sampling_radius;
    double sigma_thresh;
    double zeta;
    double time_step;
    
    double d_overshoot_;
    double bounding_radius;

    int init_iterations;
    int cutoff_iterations;
    int cache_node_threshold;
    int node_gain_threshold;
    int KFiterations;
    
    std::vector<double> boundary_min;
    std::vector<double> boundary_max;

    std::string robot_frame;
    std::string world_frame;

    bool visualize_tree;
    bool visualize_rays;
    bool visualize_exploration_area;
    bool visualize_static_and_dynamic_rays;

    double human_height;
    double human_width;
    double human_linear_velocity;
    double human_angular_velocity;

    double drone_height;
    double drone_width;
    double drone_linear_velocity;
    double drone_angular_velocity;

    double boosted_boundary_length;
    double boost_magnitude;

    double max_sampled_initial_nodes;
    double max_sampled_nodes;
    bool rrt_log_enabled;
    std::string rrt_log_path;
    std::string rrt_goal_log_path;
    int rrt_log_every_n;

    bool drone_freeze;

    double global_planner_counter;

    int look_ahead_horizon;

    // Tree-guided exploration objective
    bool tree_guidance_enabled;
    std::string tree_confirmed_topic;
    std::string tree_candidate_topic;
    double tree_gain_weight;
    double tree_candidate_bonus;
    double tree_uncertainty_conf_weight;
    double tree_uncertainty_fit_weight;
    double tree_uncertainty_age_weight;
    double tree_age_tau_sec;
    double tree_view_min_range;
    double tree_view_max_range;
    double tree_preferred_range;
    double tree_range_sigma;
    double tree_fit_error_norm;

  };

  Params readParams();
}

#endif
