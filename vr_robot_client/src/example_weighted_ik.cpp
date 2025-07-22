#include "weighted_ik.h"

int main() {
    // Create solver once with robot-specific parameters (no current_pose in constructor)
    std::array<double, 7> neutral_pose = {0.0, 0.0, 0.0, -1.5, 0.0, 1.86, 0.0};
    
    WeightedIKSolver solver(neutral_pose, 1.0, 0.5, 2.0, true);
    
    // Multiple target poses
    std::vector<std::array<double, 3>> targets = {
        {0.23189, -0.0815989, 0.607269}
    };
    
    std::array<double, 9> orientation = {
        -0.189536, 0.0420467, -0.980973,
         0.404078, -0.907217, -0.116958,
        -0.894873, -0.418557, 0.15496
    };
    
    // Simulate robot motion - current pose changes with each movement
    std::array<double, 7> current_pose = {-1.5, 0.5, 1.5, -1.5, 0.5, 0.5, 1.5};
    
    // Solve multiple targets efficiently
    for (const auto& target : targets) {
        WeightedIKResult result = solver.solve_q7(target, orientation, current_pose, 0.3, 0.5, 0.01);
        
        if (result.success) {
            cout << "Target solved successfully!" << endl;
            // Update current pose for next iteration (robot moves to new position)
            current_pose = result.joint_angles;
        }
    }
    
    return 0;
}