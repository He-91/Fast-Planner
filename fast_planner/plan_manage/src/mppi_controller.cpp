/**
* This file is part of Fast-Planner.
*
* MPPI Controller implementation for local trajectory generation
*/

#include <plan_manage/mppi_controller.h>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace fast_planner {

MPPIController::MPPIController() : 
    gen_(std::chrono::steady_clock::now().time_since_epoch().count()),
    pos_noise_(0.0, 1.0),
    vel_noise_(0.0, 1.0),
    acc_noise_(0.0, 1.0) {
}

MPPIController::~MPPIController() {}

void MPPIController::init(ros::NodeHandle& nh, EDTEnvironment::Ptr env) {
    env_ = env;
    
    // Load MPPI parameters
    nh.param("mppi/num_samples", params_.num_samples, 1000);
    nh.param("mppi/horizon_length", params_.horizon_length, 20);
    nh.param("mppi/dt", params_.dt, 0.1);
    nh.param("mppi/lambda", params_.lambda, 1.0);
    nh.param("mppi/sigma_pos", params_.sigma_pos, 0.2);
    nh.param("mppi/sigma_vel", params_.sigma_vel, 0.5);
    nh.param("mppi/sigma_acc", params_.sigma_acc, 1.0);
    
    // Cost function weights
    nh.param("mppi/w_collision", params_.w_collision, 1000.0);
    nh.param("mppi/w_goal", params_.w_goal, 10.0);
    nh.param("mppi/w_smoothness", params_.w_smoothness, 1.0);
    nh.param("mppi/w_control", params_.w_control, 0.1);
    nh.param("mppi/w_velocity", params_.w_velocity, 10.0);
    nh.param("mppi/w_acceleration", params_.w_acceleration, 5.0);
    
    // Physical limits
    nh.param("mppi/max_vel", params_.max_vel, 3.0);
    nh.param("mppi/max_acc", params_.max_acc, 2.0);
    nh.param("mppi/safe_distance", params_.safe_distance, 0.3);
    
    // Initialize noise distributions
    pos_noise_ = std::normal_distribution<double>(0.0, params_.sigma_pos);
    vel_noise_ = std::normal_distribution<double>(0.0, params_.sigma_vel);
    acc_noise_ = std::normal_distribution<double>(0.0, params_.sigma_acc);
    
    // Reserve memory for trajectories
    sample_trajectories_.reserve(params_.num_samples);
    sample_costs_.reserve(params_.num_samples);
    sample_weights_.reserve(params_.num_samples);
    
    ROS_INFO("[MPPI] Controller initialized with %d samples, horizon %d", 
             params_.num_samples, params_.horizon_length);
}

bool MPPIController::generateTrajectory(const Eigen::VectorXd& start_state,
                                      const Eigen::Vector3d& goal_pos,
                                      const std::vector<Eigen::Vector3d>& guide_path,
                                      std::vector<Eigen::Vector3d>& trajectory) {
    
    // Clear previous samples
    sample_trajectories_.clear();
    sample_costs_.clear();
    sample_weights_.clear();
    
    // Generate noise samples and rollout trajectories
    generateNoiseSamples();
    
    // Compute costs for all samples
    double min_cost = std::numeric_limits<double>::max();
    int best_sample_idx = 0;
    
    for (int i = 0; i < params_.num_samples; ++i) {
        double cost = computeTrajectoryCost(sample_trajectories_[i], goal_pos, guide_path);
        sample_costs_[i] = cost;
        
        if (cost < min_cost) {
            min_cost = cost;
            best_sample_idx = i;
        }
    }
    
    // Compute importance weights
    double max_cost = *std::max_element(sample_costs_.begin(), sample_costs_.end());
    double weight_sum = 0.0;
    
    for (int i = 0; i < params_.num_samples; ++i) {
        double normalized_cost = (sample_costs_[i] - min_cost) / (max_cost - min_cost + 1e-8);
        sample_weights_[i] = exp(-params_.lambda * normalized_cost);
        weight_sum += sample_weights_[i];
    }
    
    // Normalize weights
    for (int i = 0; i < params_.num_samples; ++i) {
        sample_weights_[i] /= weight_sum;
    }
    
    // Compute weighted average trajectory
    trajectory.clear();
    trajectory.resize(params_.horizon_length);
    velocity_traj_.clear();
    velocity_traj_.resize(params_.horizon_length);
    acceleration_traj_.clear();
    acceleration_traj_.resize(params_.horizon_length);
    
    for (int t = 0; t < params_.horizon_length; ++t) {
        trajectory[t] = Eigen::Vector3d::Zero();
        for (int i = 0; i < params_.num_samples; ++i) {
            trajectory[t] += sample_weights_[i] * sample_trajectories_[i][t];
        }
    }
    
    // Compute velocities and accelerations by finite differences
    for (int t = 0; t < params_.horizon_length; ++t) {
        if (t == 0) {
            velocity_traj_[t] = start_state.segment<3>(3); // Initial velocity
            acceleration_traj_[t] = start_state.segment<3>(6); // Initial acceleration
        } else {
            velocity_traj_[t] = (trajectory[t] - trajectory[t-1]) / params_.dt;
            if (t == 1) {
                acceleration_traj_[t] = (velocity_traj_[t] - velocity_traj_[t-1]) / params_.dt;
            } else {
                acceleration_traj_[t] = (velocity_traj_[t] - velocity_traj_[t-1]) / params_.dt;
            }
        }
    }
    
    position_traj_ = trajectory;
    
    ROS_INFO("[MPPI] Generated trajectory with cost: %f", min_cost);
    return true;
}

void MPPIController::generateNoiseSamples() {
    sample_trajectories_.resize(params_.num_samples);
    
    for (int i = 0; i < params_.num_samples; ++i) {
        sample_trajectories_[i].resize(params_.horizon_length);
        
        // Generate noise sequence for this sample
        std::vector<Eigen::Vector3d> noise_sequence(params_.horizon_length);
        for (int t = 0; t < params_.horizon_length; ++t) {
            noise_sequence[t] << pos_noise_(gen_), pos_noise_(gen_), pos_noise_(gen_);
        }
        
        // Rollout trajectory with noise
        // For now, simple integration - could be improved with proper dynamics
        Eigen::Vector3d pos = Eigen::Vector3d::Zero(); // Will be set by start state
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        
        for (int t = 0; t < params_.horizon_length; ++t) {
            // Add noise to acceleration
            acc += noise_sequence[t];
            
            // Clamp acceleration
            double acc_norm = acc.norm();
            if (acc_norm > params_.max_acc) {
                acc = acc / acc_norm * params_.max_acc;
            }
            
            // Integrate
            vel += acc * params_.dt;
            
            // Clamp velocity
            double vel_norm = vel.norm();
            if (vel_norm > params_.max_vel) {
                vel = vel / vel_norm * params_.max_vel;
            }
            
            pos += vel * params_.dt;
            
            sample_trajectories_[i][t] = pos;
        }
    }
}

void MPPIController::rolloutTrajectory(const Eigen::VectorXd& start_state,
                                     const std::vector<Eigen::Vector3d>& noise_sequence,
                                     std::vector<Eigen::Vector3d>& trajectory) {
    trajectory.resize(params_.horizon_length);
    
    Eigen::Vector3d pos = start_state.head<3>();
    Eigen::Vector3d vel = start_state.segment<3>(3);
    Eigen::Vector3d acc = start_state.segment<3>(6);
    
    for (int t = 0; t < params_.horizon_length; ++t) {
        // Add noise to acceleration
        acc += noise_sequence[t];
        
        // Clamp acceleration
        double acc_norm = acc.norm();
        if (acc_norm > params_.max_acc) {
            acc = acc / acc_norm * params_.max_acc;
        }
        
        // Integrate
        vel += acc * params_.dt;
        
        // Clamp velocity
        double vel_norm = vel.norm();
        if (vel_norm > params_.max_vel) {
            vel = vel / vel_norm * params_.max_vel;
        }
        
        pos += vel * params_.dt;
        trajectory[t] = pos;
    }
}

double MPPIController::computeTrajectoryCost(const std::vector<Eigen::Vector3d>& trajectory,
                                           const Eigen::Vector3d& goal_pos,
                                           const std::vector<Eigen::Vector3d>& guide_path) {
    double total_cost = 0.0;
    
    for (int t = 0; t < params_.horizon_length; ++t) {
        const Eigen::Vector3d& pos = trajectory[t];
        
        // Collision cost
        total_cost += params_.w_collision * computeCollisionCost(pos);
        
        // Goal cost (higher weight for later timesteps)
        double time_weight = (double)(t + 1) / params_.horizon_length;
        total_cost += params_.w_goal * time_weight * computeGoalCost(pos, goal_pos);
        
        // Smoothness cost
        if (t > 1) {
            Eigen::Vector3d vel1 = trajectory[t] - trajectory[t-1];
            Eigen::Vector3d vel2 = trajectory[t-1] - trajectory[t-2];
            Eigen::Vector3d acc = (vel1 - vel2) / params_.dt;
            total_cost += params_.w_smoothness * acc.squaredNorm();
        }
        
        // Velocity constraint cost
        if (t > 0) {
            Eigen::Vector3d vel = (trajectory[t] - trajectory[t-1]) / params_.dt;
            double vel_excess = std::max(0.0, vel.norm() - params_.max_vel);
            total_cost += params_.w_velocity * vel_excess * vel_excess;
        }
    }
    
    // Path following cost
    if (!guide_path.empty()) {
        total_cost += computePathFollowingCost(trajectory, guide_path);
    }
    
    return total_cost;
}

double MPPIController::computeCollisionCost(const Eigen::Vector3d& pos) {
    double dist = env_->evaluateCoarseEDT(pos, -1);
    if (dist < params_.safe_distance) {
        double penalty = params_.safe_distance - dist;
        return penalty * penalty;
    }
    return 0.0;
}

double MPPIController::computeGoalCost(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal) {
    return (pos - goal).squaredNorm();
}

double MPPIController::computePathFollowingCost(const std::vector<Eigen::Vector3d>& trajectory,
                                              const std::vector<Eigen::Vector3d>& guide_path) {
    if (guide_path.empty()) return 0.0;
    
    double total_cost = 0.0;
    
    for (const auto& traj_point : trajectory) {
        // Find closest point on guide path
        double min_dist_sq = std::numeric_limits<double>::max();
        for (const auto& guide_point : guide_path) {
            double dist_sq = (traj_point - guide_point).squaredNorm();
            min_dist_sq = std::min(min_dist_sq, dist_sq);
        }
        total_cost += min_dist_sq;
    }
    
    return total_cost / trajectory.size();
}

double MPPIController::computeSmoothnessCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double cost = 0.0;
    for (int t = 2; t < trajectory.size(); ++t) {
        Eigen::Vector3d vel1 = trajectory[t] - trajectory[t-1];
        Eigen::Vector3d vel2 = trajectory[t-1] - trajectory[t-2];
        Eigen::Vector3d acc = (vel1 - vel2) / params_.dt;
        cost += acc.squaredNorm();
    }
    return cost;
}

void MPPIController::updateControl() {
    // This method could be used for more sophisticated control updates
    // For now, the weighted average computation is done in generateTrajectory
}

} // namespace fast_planner