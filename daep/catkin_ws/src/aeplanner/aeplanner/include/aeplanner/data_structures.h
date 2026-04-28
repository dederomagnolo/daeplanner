#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <ros/ros.h>
#include <octomap/OcTree.h>
#include <aeplanner/param.h>

namespace aeplanner
{
extern Params params_;

class RRTNode
{
public:
  Eigen::Vector4d state_;
  RRTNode* parent_;
  std::vector<RRTNode*> children_;
  double gain_;
  double dynamic_gain_;
  double base_dynamic_gain_;
  double tree_gain_raw_;
  double tree_gain_weighted_;
  int tree_best_id_;
  double tree_best_distance_;
  double tree_best_confidence_;
  double tree_best_contribution_;
  bool tree_best_confirmed_;
  int tree_total_count_;
  int tree_considered_count_;
  int tree_confirmed_considered_;
  int tree_candidate_considered_;
  double dfm_score_;
  bool gain_explicitly_calculated_;

  RRTNode()
      : parent_(NULL)
      , gain_(0.0)
      , dynamic_gain_(0.0)
      , base_dynamic_gain_(0.0)
      , tree_gain_raw_(0.0)
      , tree_gain_weighted_(0.0)
      , tree_best_id_(-1)
      , tree_best_distance_(0.0)
      , tree_best_confidence_(0.0)
      , tree_best_contribution_(0.0)
      , tree_best_confirmed_(false)
      , tree_total_count_(0)
      , tree_considered_count_(0)
      , tree_confirmed_considered_(0)
      , tree_candidate_considered_(0)
      , dfm_score_(0.0)
      , gain_explicitly_calculated_(false)
  {
  }

  ~RRTNode()
  {
    for (typename std::vector<RRTNode*>::iterator node_it = children_.begin();
         node_it != children_.end(); ++node_it)
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  RRTNode* getCopyOfParentBranch()
  {
    RRTNode* current_node = this;
    RRTNode* current_child_node = NULL;
    RRTNode* new_node;
    RRTNode* new_child_node = NULL;

    while (current_node)
    {
      new_node = new RRTNode();
      new_node->state_ = current_node->state_;
      new_node->gain_ = current_node->gain_;
      new_node->dynamic_gain_ = current_node->dynamic_gain_;
      new_node->base_dynamic_gain_ = current_node->base_dynamic_gain_;
      new_node->tree_gain_raw_ = current_node->tree_gain_raw_;
      new_node->tree_gain_weighted_ = current_node->tree_gain_weighted_;
      new_node->tree_best_id_ = current_node->tree_best_id_;
      new_node->tree_best_distance_ = current_node->tree_best_distance_;
      new_node->tree_best_confidence_ = current_node->tree_best_confidence_;
      new_node->tree_best_contribution_ = current_node->tree_best_contribution_;
      new_node->tree_best_confirmed_ = current_node->tree_best_confirmed_;
      new_node->tree_total_count_ = current_node->tree_total_count_;
      new_node->tree_considered_count_ = current_node->tree_considered_count_;
      new_node->tree_confirmed_considered_ = current_node->tree_confirmed_considered_;
      new_node->tree_candidate_considered_ = current_node->tree_candidate_considered_;
      new_node->dfm_score_ = current_node->dfm_score_;
      new_node->gain_explicitly_calculated_ = current_node->gain_explicitly_calculated_;
      new_node->parent_ = NULL;

      if (new_child_node)
      {
        new_node->children_.push_back(new_child_node);
        new_child_node->parent_ = new_node;
      }

      current_child_node = current_node;
      current_node = current_node->parent_;
      new_child_node = new_node;
    }

    return new_node;
  }

  // Dynamic score
  double dynamic_score(double lambda, double zeta) const
  { 
    // zeta * frequency map
    if (this->parent_){
      return this->parent_->dynamic_score(lambda, zeta) +
             this->dynamic_gain_ * exp(-lambda * this->distance(this->parent_) * (1 + (zeta * this->dfm_score_)));
             }
    else  {
      return this->dynamic_gain_ * (1 + (zeta * this->dfm_score_));     
    } 
  }

  // Static score
  double score(double lambda)
  { 
    if (this->parent_)
      return this->parent_->score(lambda) +
             this->gain_ * exp(-lambda * this->distance(this->parent_));
    else
      return this->gain_;
  }

  double cost()
  {
    if (this->parent_)
      return this->distance(this->parent_) + this->parent_->cost();
    else
      return 0;
  }

  double distance(RRTNode* other) const
  {
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    return (p3 - q3).norm();
  }

  double time_cost()
  {
    if (this->parent_)
      return this->time_to_reach(this->parent_) + this->parent_->time_cost();
    else
      return 0;
  }

  double time_to_reach(RRTNode* other) {
    double current_x = this->state_[0];
    double current_y = this->state_[1];
    double current_z = this->state_[2];
    double current_yaw = this->state_[3];

    double target_x = other->state_[0];
    double target_y = other->state_[1];
    double target_z = other->state_[2];
    double target_yaw = other->state_[3];

    // Calculate yaw angle difference
    double yaw_difference = target_yaw - current_yaw;
    if (yaw_difference > M_PI) {
        yaw_difference -= 2 * M_PI;
    } else if (yaw_difference < -M_PI) {
        yaw_difference += 2 * M_PI;
    }

    // Calculate time to rotate to target yaw angle
    double rotation_time = fabs(yaw_difference) / params_.drone_angular_velocity;

    // Calculate Euclidean distance in x-y plane
    Eigen::Vector3d p3(this->state_[0], this->state_[1], this->state_[2]);
    Eigen::Vector3d q3(other->state_[0], other->state_[1], other->state_[2]);
    double euclidean_distance = (p3 - q3).norm();

    // Calculate time to move linearly to target position
    double linear_time = euclidean_distance / params_.drone_linear_velocity;

    // Calculate total time to reach target state
    double total_time = rotation_time + linear_time;

    return total_time;
  }
};
}  // namespace aeplanner

#endif
