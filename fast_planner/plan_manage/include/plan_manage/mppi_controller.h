/**
* This file is part of Fast-Planner.
*
* MPPI Controller implementation for local trajectory generation
*/

#ifndef _MPPI_CONTROLLER_H_
#define _MPPI_CONTROLLER_H_

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>

namespace fast_planner {

struct MPPIParams {
    int num_samples;        // Number of trajectory samples
    int horizon_length;     // Planning horizon steps
    double dt;              // Time step
    double lambda;          // Inverse temperature for importance sampling
    double sigma_pos;       // Position noise standard deviation
    double sigma_vel;       // Velocity noise standard deviation
    double sigma_acc;       // Acceleration noise standard deviation
    
    // Cost function weights
    double w_collision;     // Collision cost weight
    double w_goal;          // Goal reaching cost weight  
    double w_smoothness;    // Trajectory smoothness weight
    double w_control;       // Control effort weight
    double w_velocity;      // Velocity limit weight
    double w_acceleration;  // Acceleration limit weight
    
    // Physical limits
    double max_vel;
    double max_acc;
    double safe_distance;   // Minimum distance to obstacles
};

class MPPIController {
public:
    MPPIController();
    ~MPPIController();
    
    void init(ros::NodeHandle& nh, EDTEnvironment::Ptr env);
    
    /**
     * @brief Generate optimal trajectory using MPPI
     * @param start_state Current state [pos, vel, acc]
     * @param goal_pos Goal position
     * @param guide_path Guide path from topology planning
     * @param trajectory Output trajectory as sequence of positions
     * @return True if successful trajectory found
     */
    bool generateTrajectory(const Eigen::VectorXd& start_state,
                          const Eigen::Vector3d& goal_pos,
                          const std::vector<Eigen::Vector3d>& guide_path,
                          std::vector<Eigen::Vector3d>& trajectory);
    
    /**
     * @brief Get velocity trajectory
     */
    std::vector<Eigen::Vector3d> getVelocityTrajectory() const { return velocity_traj_; }
    
    /**
     * @brief Get acceleration trajectory
     */
    std::vector<Eigen::Vector3d> getAccelerationTrajectory() const { return acceleration_traj_; }
    
    /**
     * @brief Get trajectory duration
     */
    double getTrajectoryDuration() const { return params_.horizon_length * params_.dt; }

private:
    MPPIParams params_;
    EDTEnvironment::Ptr env_;
    
    // Random number generation
    std::default_random_engine gen_;
    std::normal_distribution<double> pos_noise_;
    std::normal_distribution<double> vel_noise_;
    std::normal_distribution<double> acc_noise_;
    
    // Trajectory storage
    std::vector<Eigen::Vector3d> position_traj_;
    std::vector<Eigen::Vector3d> velocity_traj_;
    std::vector<Eigen::Vector3d> acceleration_traj_;
    
    // Sample trajectories and costs
    std::vector<std::vector<Eigen::Vector3d>> sample_trajectories_;
    std::vector<double> sample_costs_;
    std::vector<double> sample_weights_;
    
    /**
     * @brief Generate noise samples for MPPI
     */
    void generateNoiseSamples();
    
    /**
     * @brief Rollout trajectory with noise
     */
    void rolloutTrajectory(const Eigen::VectorXd& start_state,
                          const std::vector<Eigen::Vector3d>& noise_sequence,
                          std::vector<Eigen::Vector3d>& trajectory);
    
    /**
     * @brief Compute trajectory cost
     */
    double computeTrajectoryCost(const std::vector<Eigen::Vector3d>& trajectory,
                               const Eigen::Vector3d& goal_pos,
                               const std::vector<Eigen::Vector3d>& guide_path);
    
    /**
     * @brief Compute collision cost for a position
     */
    double computeCollisionCost(const Eigen::Vector3d& pos);
    
    /**
     * @brief Compute goal reaching cost
     */
    double computeGoalCost(const Eigen::Vector3d& pos, const Eigen::Vector3d& goal);
    
    /**
     * @brief Compute path following cost
     */
    double computePathFollowingCost(const std::vector<Eigen::Vector3d>& trajectory,
                                  const std::vector<Eigen::Vector3d>& guide_path);
    
    /**
     * @brief Compute trajectory smoothness cost
     */
    double computeSmoothnessCost(const std::vector<Eigen::Vector3d>& trajectory);
    
    /**
     * @brief Update control using weighted average of samples
     */
    void updateControl();
};

} // namespace fast_planner

#endif // _MPPI_CONTROLLER_H_