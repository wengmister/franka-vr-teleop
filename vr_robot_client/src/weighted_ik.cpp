#include "weighted_ik.h"

// Constructor - only robot-specific parameters
WeightedIKSolver::WeightedIKSolver(
    const std::array<double, 7>& neutral_pose,
    double weight_manip,
    double weight_neutral,
    double weight_current,
    bool verbose
) : neutral_pose_(neutral_pose),
    weight_manip_(weight_manip),
    weight_neutral_(weight_neutral),
    weight_current_(weight_current),
    verbose_(verbose) {
    
    // Pre-compute normalization factor
    normalization_factor_ = 7.0 * 6.28;
}

double WeightedIKSolver::calculate_manipulability(const std::array<std::array<double, 6>, 7>& J) const {
    // Convert array to Eigen matrix
    Eigen::MatrixXd jacobian(6, 7);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 7; j++) {
            jacobian(i, j) = J[j][i];
        }
    }
    
    // Calculate manipulability as sqrt(det(J * J^T))
    Eigen::MatrixXd JJT = jacobian * jacobian.transpose();
    double det = JJT.determinant();
    
    return (det >= 0) ? sqrt(det) : 0.0;
}

double WeightedIKSolver::calculate_distance(const std::array<double, 7>& q1, const std::array<double, 7>& q2) const {
    double distance = 0.0;
    for (int j = 0; j < 7; j++) {
        double diff = q1[j] - q2[j];
        distance += diff * diff;
    }
    return sqrt(distance);
}

double WeightedIKSolver::compute_score(double manipulability, double neutral_dist, double current_dist) const {
    double normalized_neutral_dist = neutral_dist / normalization_factor_;
    double normalized_current_dist = current_dist / normalization_factor_;
    
    return weight_manip_ * manipulability 
         - weight_neutral_ * normalized_neutral_dist 
         - weight_current_ * normalized_current_dist;
}

WeightedIKResult WeightedIKSolver::solve_q7(
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& current_pose,  // Now a parameter
    double q7_start,
    double q7_end,
    double step_size
) {
    WeightedIKResult result;
    result.success = false;
    result.score = -std::numeric_limits<double>::infinity();
    result.total_solutions_found = 0;
    result.valid_solutions_count = 0;
    result.q7_values_tested = (int)((q7_end - q7_start) / step_size) + 1;
    
    // Variables for IK solving
    unsigned int nsols = 0;
    bool joint_angles = true;
    std::array<std::array<double, 7>, 8> qsols;
    std::array<std::array<std::array<double, 6>, 7>, 8> Jsols;
    
    if (verbose_) {
        cout << endl << "=======================================================" << endl;
        cout << "Weighted IK Q7 Optimization (Class-based)" << endl;
        cout << "=======================================================" << endl;
        cout << "Target position: [" << target_position[0] << ", " << target_position[1] << ", " << target_position[2] << "]" << endl;
        cout << "Q7 range: " << q7_start << " to " << q7_end << " rad (step: " << step_size << ")" << endl;
        cout << "Weights - Manipulability: " << weight_manip_ << ", Neutral: " << weight_neutral_ << ", Current: " << weight_current_ << endl;
        cout << endl;
    }
    
    auto start = high_resolution_clock::now();
    
    // Sweep through q7 values
    for (double q7_sweep = q7_start; q7_sweep <= q7_end; q7_sweep += step_size) {
        nsols = franka_J_ik_q7(target_position, target_orientation, q7_sweep, Jsols, qsols, joint_angles);
        result.total_solutions_found += nsols;
        
        // Check each solution for this q7 value
        for (int i = 0; i < nsols; i++) {
            // Check if solution is valid (all joints within limits)
            bool valid_solution = true;
            for (int j = 0; j < 7; j++) {
                if (isnan(qsols[i][j])) {
                    valid_solution = false;
                    break;
                }
            }
            
            if (valid_solution) {
                result.valid_solutions_count++;
                
                // Calculate metrics using current_pose parameter
                double manipulability = calculate_manipulability(Jsols[i]);
                double neutral_distance = calculate_distance(qsols[i], neutral_pose_);
                double current_distance = calculate_distance(qsols[i], current_pose);
                double score = compute_score(manipulability, neutral_distance, current_distance);
                
                // Update best solution if this one is better
                if (score > result.score) {
                    result.success = true;
                    result.score = score;
                    result.manipulability = manipulability;
                    result.neutral_distance = neutral_distance;
                    result.current_distance = current_distance;
                    result.q7_optimal = q7_sweep;
                    result.joint_angles = qsols[i];
                    result.jacobian = Jsols[i];
                    result.solution_index = i;
                }
            }
        }
    }
    
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    result.duration_microseconds = duration.count();
    
    if (verbose_) {
        print_weighted_ik_results(result);
    }
    
    return result;
}

void WeightedIKSolver::update_weights(double weight_manip, double weight_neutral, double weight_current) {
    weight_manip_ = weight_manip;
    weight_neutral_ = weight_neutral;
    weight_current_ = weight_current;
}

void WeightedIKSolver::update_neutral_pose(const std::array<double, 7>& neutral_pose) {
    neutral_pose_ = neutral_pose;
}

// Keep original function for backward compatibility
WeightedIKResult weighted_ik_q7(
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& neutral_pose,
    const std::array<double, 7>& current_pose,
    double q7_start, double q7_end, double step_size,
    double weight_manip, double weight_neutral, double weight_current,
    bool verbose
) {
    WeightedIKSolver solver(neutral_pose, weight_manip, weight_neutral, weight_current, verbose);
    return solver.solve_q7(target_position, target_orientation, current_pose, q7_start, q7_end, step_size);
}

void print_weighted_ik_results(const WeightedIKResult& result) {
    cout << "Optimization completed!" << endl;
    cout << "Q7 values tested: " << result.q7_values_tested << endl;
    cout << "Total solutions found: " << result.total_solutions_found << endl;
    cout << "Valid solutions: " << result.valid_solutions_count << endl;
    cout << "Duration: " << result.duration_microseconds << " microseconds (" 
         << result.duration_microseconds / 1000.0 << " milliseconds)" << endl;
    cout << endl;
    
    if (result.success) {
        cout << "OPTIMAL SOLUTION FOUND:" << endl;
        cout << "Optimal q7: " << result.q7_optimal << " rad" << endl;
        cout << "Overall score: " << std::setprecision(8) << result.score << endl;
        cout << "Solution index: " << result.solution_index + 1 << endl;
        cout << endl;
        
        cout << "Solution metrics:" << endl;
        cout << "  Manipulability: " << std::setprecision(6) << result.manipulability << endl;
        cout << "  Distance from neutral: " << std::setprecision(6) << result.neutral_distance << " rad" << endl;
        cout << "  Distance from current: " << std::setprecision(6) << result.current_distance << " rad" << endl;
        cout << endl;
        
        cout << "Joint angles (radians):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << result.joint_angles[j] << endl;
        }
        cout << endl;
        
        cout << "Joint angles (degrees):" << endl;
        for (int j = 0; j < 7; j++) {
            cout << "q_" << j + 1 << " = " << std::setprecision(6) << result.joint_angles[j] * 180 / PI << endl;
        }
        cout << endl;
        
        // Forward kinematics verification
        Eigen::Matrix4d T_best = franka_fk(result.joint_angles);
        cout << "Forward kinematics verification:" << endl;
        cout << T_best << endl;
        
    } else {
        cout << "No valid solutions found in the specified q7 range!" << endl;
    }
}