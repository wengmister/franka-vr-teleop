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

double WeightedIKSolver::evaluate_q7_cost(
    double q7,
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& current_pose
) const {
    // Variables for IK solving
    unsigned int nsols = 0;
    bool joint_angles = true;
    std::array<std::array<double, 7>, 8> qsols;
    std::array<std::array<std::array<double, 6>, 7>, 8> Jsols;
    
    // Solve IK for this q7 value
    nsols = franka_J_ik_q7(target_position, target_orientation, q7, Jsols, qsols, joint_angles);
    
    if (nsols == 0) {
        // No solutions found for this q7
        return -std::numeric_limits<double>::infinity();
    }
    
    double best_score = -std::numeric_limits<double>::infinity();
    
    // Evaluate each solution
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
            // Calculate metrics
            double manipulability = calculate_manipulability(Jsols[i]);
            double neutral_distance = calculate_distance(qsols[i], neutral_pose_);
            double current_distance = calculate_distance(qsols[i], current_pose);
            double score = compute_score(manipulability, neutral_distance, current_distance);
            
            // Update best score for this q7
            if (score > best_score) {
                best_score = score;
            }
        }
    }
    
    return best_score;
}

double WeightedIKSolver::brent_optimize(
    double ax, double bx, double cx,
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& current_pose,
    double tolerance,
    int max_iterations,
    int& iterations_used
) const {
    const double CGOLD = 0.3819660;  // Golden ratio constant
    const double TINY = 1e-20;       // Small number to avoid division by zero
    
    double a, b, d = 0.0, e = 0.0, etemp, fu, fv, fw, fx;
    double p, q, r, tol1, tol2, u, v, w, x, xm;
    
    // Initialize bounds
    a = (ax < cx ? ax : cx);
    b = (ax > cx ? ax : cx);
    
    // Initialize points
    x = w = v = bx;
    fw = fv = fx = -evaluate_q7_cost(x, target_position, target_orientation, current_pose);  // Minimize negative of cost
    
    iterations_used = 0;
    
    for (int iter = 1; iter <= max_iterations; iter++) {
        iterations_used = iter;
        
        xm = 0.5 * (a + b);
        tol2 = 2.0 * (tol1 = tolerance * fabs(x) + TINY);
        
        // Check convergence
        if (fabs(x - xm) <= (tol2 - 0.5 * (b - a))) {
            return x;  // Converged
        }
        
        if (fabs(e) > tol1) {
            // Construct parabolic fit
            r = (x - w) * (fx - fv);
            q = (x - v) * (fx - fw);
            p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if (q > 0.0) p = -p;
            q = fabs(q);
            etemp = e;
            e = d;
            
            // Check if parabolic fit is acceptable
            if (fabs(p) >= fabs(0.5 * q * etemp) || p <= q * (a - x) || p >= q * (b - x)) {
                // Golden section step
                d = CGOLD * (e = (x >= xm ? a - x : b - x));
            } else {
                // Parabolic step
                d = p / q;
                u = x + d;
                if (u - a < tol2 || b - u < tol2) {
                    d = (xm - x >= 0 ? fabs(tol1) : -fabs(tol1));
                }
            }
        } else {
            // Golden section step
            d = CGOLD * (e = (x >= xm ? a - x : b - x));
        }
        
        // Function evaluation
        u = (fabs(d) >= tol1 ? x + d : x + (d >= 0 ? fabs(tol1) : -fabs(tol1)));
        fu = -evaluate_q7_cost(u, target_position, target_orientation, current_pose);  // Minimize negative of cost
        
        // Update points
        if (fu <= fx) {
            if (u >= x) a = x; else b = x;
            v = w; w = x; x = u;
            fv = fw; fw = fx; fx = fu;
        } else {
            if (u < x) a = u; else b = u;
            if (fu <= fw || w == x) {
                v = w; w = u;
                fv = fw; fw = fu;
            } else if (fu <= fv || v == x || v == w) {
                v = u;
                fv = fu;
            }
        }
    }
    
    // Maximum iterations reached
    return x;
}

WeightedIKResult WeightedIKSolver::solve_q7_optimized(
    const std::array<double, 3>& target_position,
    const std::array<double, 9>& target_orientation,
    const std::array<double, 7>& current_pose,
    double q7_min,
    double q7_max,
    double tolerance,
    int max_iterations
) {
    WeightedIKResult result;
    result.success = false;
    result.score = -std::numeric_limits<double>::infinity();
    result.total_solutions_found = 0;
    result.valid_solutions_count = 0;
    result.q7_values_tested = 0;  // Will be set to optimization iterations
    result.optimization_iterations = 0;
    
    if (verbose_) {
        cout << endl << "=======================================================" << endl;
        cout << "Weighted IK Q7 Optimization (1D Optimization)" << endl;
        cout << "=======================================================" << endl;
        cout << "Target position: [" << target_position[0] << ", " << target_position[1] << ", " << target_position[2] << "]" << endl;
        cout << "Q7 range: " << q7_min << " to " << q7_max << " rad" << endl;
        cout << "Tolerance: " << tolerance << endl;
        cout << "Max iterations: " << max_iterations << endl;
        cout << "Weights - Manipulability: " << weight_manip_ << ", Neutral: " << weight_neutral_ << ", Current: " << weight_current_ << endl;
        cout << endl;
    }
    
    auto start = high_resolution_clock::now();
    
    // Use Brent's method to find optimal q7
    // We need three initial points: ax, bx, cx where bx is between ax and cx
    double ax = q7_min;
    double cx = q7_max;
    double bx = 0.5 * (ax + cx);  // Start in the middle
    
    int iterations_used = 0;
    double optimal_q7 = brent_optimize(ax, bx, cx, target_position, target_orientation, current_pose,
                                      tolerance, max_iterations, iterations_used);
    
    result.optimization_iterations = iterations_used;
    result.q7_values_tested = iterations_used;  // For compatibility
    
    // Now evaluate the optimal q7 to get full solution details
    unsigned int nsols = 0;
    bool joint_angles = true;
    std::array<std::array<double, 7>, 8> qsols;
    std::array<std::array<std::array<double, 6>, 7>, 8> Jsols;
    
    nsols = franka_J_ik_q7(target_position, target_orientation, optimal_q7, Jsols, qsols, joint_angles);
    result.total_solutions_found = nsols;
    
    if (nsols > 0) {
        // Find the best solution for the optimal q7
        for (int i = 0; i < nsols; i++) {
            // Check if solution is valid
            bool valid_solution = true;
            for (int j = 0; j < 7; j++) {
                if (isnan(qsols[i][j])) {
                    valid_solution = false;
                    break;
                }
            }
            
            if (valid_solution) {
                result.valid_solutions_count++;
                
                // Calculate metrics
                double manipulability = calculate_manipulability(Jsols[i]);
                double neutral_distance = calculate_distance(qsols[i], neutral_pose_);
                double current_distance = calculate_distance(qsols[i], current_pose);
                double score = compute_score(manipulability, neutral_distance, current_distance);
                
                // Update best solution
                if (score > result.score) {
                    result.success = true;
                    result.score = score;
                    result.manipulability = manipulability;
                    result.neutral_distance = neutral_distance;
                    result.current_distance = current_distance;
                    result.q7_optimal = optimal_q7;
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
        cout << "Optimization completed!" << endl;
        cout << "Optimization iterations: " << result.optimization_iterations << endl;
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
    
    return result;
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