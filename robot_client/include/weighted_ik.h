#ifndef WEIGHTED_IK_H
#define WEIGHTED_IK_H

#include <array>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <limits>
#include <cmath>
#include "Eigen/Dense"
#include "geofik.h"

using namespace std;
using namespace std::chrono;

// Structure to hold the result of weighted IK optimization
struct WeightedIKResult {
    bool success;
    std::array<double, 7> joint_angles;
    double q7_optimal;
    double score;
    double manipulability;
    double neutral_distance;
    double current_distance;
    int solution_index;
    std::array<std::array<double, 6>, 7> jacobian;
    
    int total_solutions_found;
    int valid_solutions_count;
    int q7_values_tested;
    int optimization_iterations;  // Number of iterations used by optimization algorithm
    long duration_microseconds;
};

class WeightedIKSolver {
private:
    // Franka joint ranges (radians) based on datasheet limits
    static constexpr std::array<double, 7> JOINT_RANGES = {{
        5.796,  // A1: [-166°, 166°] = [-2.898, 2.898] rad
        3.524,  // A2: [-101°, 101°] = [-1.762, 1.762] rad  
        5.796,  // A3: [-166°, 166°] = [-2.898, 2.898] rad
        3.001,  // A4: [-176°, -4°] = [-3.071, -0.070] rad
        5.796,  // A5: [-166°, 166°] = [-2.898, 2.898] rad
        3.769,  // A6: [-1°, 215°] = [-0.017, 3.752] rad
        5.796   // A7: [-166°, 166°] = [-2.898, 2.898] rad
    }};

    // Pre-configured parameters (robot-specific, don't change)
    std::array<double, 7> neutral_pose_;
    double weight_manip_;
    double weight_neutral_;
    double weight_current_;
    
    // Per-joint weights for current distance calculation (for base stabilization)
    std::array<double, 7> joint_weights_;
    
    bool verbose_;
    
    // Helper methods
    double calculate_manipulability(const std::array<std::array<double, 6>, 7>& J) const;
    double calculate_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const;
    double calculate_weighted_current_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const;
    double calculate_normalized_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const;
    double calculate_normalized_weighted_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const;
    double compute_score(double manipulability, double neutral_dist, double current_dist) const;
    
    // Cost function for optimization
    double evaluate_q7_cost(
        double q7,
        const std::array<double, 3>& target_position,
        const std::array<double, 9>& target_orientation,
        const std::array<double, 7>& current_pose
    ) const;
    
    // 1D optimization algorithms
    double brent_optimize(
        double ax, double bx, double cx,
        const std::array<double, 3>& target_position,
        const std::array<double, 9>& target_orientation,
        const std::array<double, 7>& current_pose,
        double tolerance,
        int max_iterations,
        int& iterations_used
    ) const;

public:
    // Constructor - only takes robot-specific parameters that don't change
    WeightedIKSolver(
        const std::array<double, 7>& neutral_pose,
        double weight_manip,
        double weight_neutral,
        double weight_current,
        const std::array<double, 7>& joint_weights = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
        bool verbose = true
    );
    
    // Main solving method using 1D optimization
    WeightedIKResult solve_q7_optimized(
        const std::array<double, 3>& target_position,
        const std::array<double, 9>& target_orientation,
        const std::array<double, 7>& current_pose,  // Current robot state
        double q7_min,
        double q7_max,
        double tolerance = 1e-6,
        int max_iterations = 100
    );
    
    // Update weights without recreating object
    void update_weights(double weight_manip, double weight_neutral, double weight_current);
    
    // Update joint weights for base stabilization
    void update_joint_weights(const std::array<double, 7>& joint_weights);
    
    // Update neutral pose (rarely needed)
    void update_neutral_pose(const std::array<double, 7>& neutral_pose);
    
    // Getters
    const std::array<double, 7>& get_neutral_pose() const { return neutral_pose_; }
    void set_verbose(bool verbose) { verbose_ = verbose; }
};


#endif // WEIGHTED_IK_H