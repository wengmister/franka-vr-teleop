#ifndef WEIGHTED_IK_H
#define WEIGHTED_IK_H

#include <array>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <limits>
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
    long duration_microseconds;
};

class WeightedIKSolver {
private:
    // Pre-configured parameters (robot-specific, don't change)
    std::array<double, 7> neutral_pose_;
    double weight_manip_;
    double weight_neutral_;
    double weight_current_;
    
    // Pre-computed constants
    double normalization_factor_;
    bool verbose_;
    
    // Helper methods
    double calculate_manipulability(const std::array<std::array<double, 6>, 7>& J) const;
    double calculate_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const;
    double compute_score(double manipulability, double neutral_dist, double current_dist) const;

public:
    // Constructor - only takes robot-specific parameters that don't change
    WeightedIKSolver(
        const std::array<double, 7>& neutral_pose,
        double weight_manip,
        double weight_neutral,
        double weight_current,
        bool verbose = true
    );
    
    // Main solving method - current_pose is passed in as it changes with robot motion
    WeightedIKResult solve_q7(
        const std::array<double, 3>& target_position,
        const std::array<double, 9>& target_orientation,
        const std::array<double, 7>& current_pose,  // Current robot state
        double q7_start,
        double q7_end,
        double step_size
    );
    
    // Update weights without recreating object
    void update_weights(double weight_manip, double weight_neutral, double weight_current);
    
    // Update neutral pose (rarely needed)
    void update_neutral_pose(const std::array<double, 7>& neutral_pose);
    
    // Getters
    const std::array<double, 7>& get_neutral_pose() const { return neutral_pose_; }
    void set_verbose(bool verbose) { verbose_ = verbose; }
};

// Standalone functions for backward compatibility
double calculate_manipulability_weighted(const std::array<std::array<double, 6>, 7>& J);
WeightedIKResult weighted_ik_q7(
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& neutral_pose,
    const std::array<double, 7>& current_pose,
    double q7_start, double q7_end, double step_size,
    double weight_manip, double weight_neutral, double weight_current,
    bool verbose = true
);
void print_weighted_ik_results(const WeightedIKResult& result);

#endif // WEIGHTED_IK_H