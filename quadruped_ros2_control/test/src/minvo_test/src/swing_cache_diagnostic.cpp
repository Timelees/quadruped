/**
 * @file swing_cache_diagnostic.cpp
 * @brief Diagnostic test to verify the swing trajectory caching behavior.
 *
 * This test simulates the scenario where:
 * 1. A swing is planned for the future (swingStartTime > initTime)
 * 2. On the next MPC iteration, the same swing should be cached
 * 3. But currently, it's re-solved with different touchDown positions
 *
 * Build and run:
 *   colcon build --packages-select minvo_test --cmake-args -DCMAKE_BUILD_TYPE=Release
 *   ros2 run minvo_test swing_cache_diagnostic
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

// Simulate the caching logic from SwingTrajectoryPlanner
struct OptimizedTrajectory {
    void* bspline = nullptr;  // Simplified: just a pointer to check existence
    double startTime = 0.0;
    double endTime = 0.0;
    bool solved = false;

    bool valid() const { return solved && bspline != nullptr; }
};

struct SwingPlannerState {
    OptimizedTrajectory active;
};

// Simulates the current caching logic
bool shouldReSolveCurrent(const SwingPlannerState& state,
                          double swingStartTime, double swingFinalTime,
                          double initTime) {
    const bool executing = swingStartTime <= initTime;
    const auto& active = state.active;

    // Current freeze check (line 291-292 in SwingTrajectoryPlanner.cpp)
    if (executing && active.bspline && active.startTime == swingStartTime &&
        active.endTime == swingFinalTime) {
        return false;  // Frozen, don't re-solve
    }
    return true;  // Re-solve
}

// Simulates the proposed caching logic
bool shouldReSolveProposed(const SwingPlannerState& state,
                           double swingStartTime, double swingFinalTime,
                           double initTime) {
    const bool executing = swingStartTime <= initTime;
    const auto& active = state.active;

    // Check if we have a cached trajectory for this swing
    if (active.bspline && active.startTime == swingStartTime &&
        active.endTime == swingFinalTime) {
        // For executing swings: always freeze
        if (executing) {
            return false;
        }
        // For future swings: freeze only if we've already solved once
        // (This prevents continuous re-optimization as footholds change)
        if (active.solved) {
            return false;
        }
    }
    return true;
}

void updateCacheCurrent(SwingPlannerState& state, double swingStartTime,
                        double swingFinalTime, double initTime, bool qpSuccess) {
    auto& active = state.active;
    // Current cache condition (line 326 and 338-339)
    if (!active.valid() || active.endTime <= initTime || swingStartTime <= active.startTime) {
        if (qpSuccess) {
            active.bspline = reinterpret_cast<void*>(0x1);  // Dummy pointer
            active.startTime = swingStartTime;
            active.endTime = swingFinalTime;
            active.solved = true;
        } else {
            active.bspline = reinterpret_cast<void*>(0x1);
            active.startTime = swingStartTime;
            active.endTime = swingFinalTime;
            active.solved = false;
        }
    }
}

void updateCacheProposed(SwingPlannerState& state, double swingStartTime,
                         double swingFinalTime, double initTime, bool qpSuccess) {
    auto& active = state.active;
    // Proposed: always update cache for the next swing
    // Only update if this is a new swing or the first time solving this swing
    if (active.startTime != swingStartTime || active.endTime != swingFinalTime ||
        !active.bspline) {
        if (qpSuccess) {
            active.bspline = reinterpret_cast<void*>(0x1);
            active.startTime = swingStartTime;
            active.endTime = swingFinalTime;
            active.solved = true;
        } else {
            active.bspline = reinterpret_cast<void*>(0x1);
            active.startTime = swingStartTime;
            active.endTime = swingFinalTime;
            active.solved = false;
        }
    }
}

int main() {
    std::cout << "=== Swing Trajectory Cache Diagnostic ===\n\n";

    // Scenario: Trot gait, RF leg about to start swinging
    // Swing 1: [0.3, 0.6], Swing 2: [0.6, 0.9], ...
    const double swing1Start = 0.3, swing1End = 0.6;

    std::cout << "Scenario: Robot enters trot at t=0.3, RF leg starts swinging\n";
    std::cout << "First swing: [" << swing1Start << ", " << swing1End << "]\n\n";

    // Test current logic
    std::cout << "--- Current Logic ---\n";
    {
        SwingPlannerState state;
        double initTime = 0.25;  // Before swing starts

        // MPC iteration 1: t=0.25, planning for swing [0.3, 0.6]
        std::cout << "MPC iter 1 (t=" << initTime << "): ";
        bool reSolve1 = shouldReSolveCurrent(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve1 ? "YES" : "NO (cached)") << "\n";
        if (reSolve1) {
            updateCacheCurrent(state, swing1Start, swing1End, initTime, true);
            std::cout << "  -> QP solved, cache updated\n";
        }

        // MPC iteration 2: t=0.28, same swing [0.3, 0.6]
        initTime = 0.28;
        std::cout << "MPC iter 2 (t=" << initTime << "): ";
        bool reSolve2 = shouldReSolveCurrent(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve2 ? "YES" : "NO (cached)") << "\n";
        if (reSolve2) {
            std::cout << "  -> PROBLEM: Re-solving future swing with new footholds!\n";
            updateCacheCurrent(state, swing1Start, swing1End, initTime, true);
        }

        // MPC iteration 3: t=0.32, swing is now executing
        initTime = 0.32;
        std::cout << "MPC iter 3 (t=" << initTime << "): ";
        bool reSolve3 = shouldReSolveCurrent(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve3 ? "YES" : "NO (cached)") << "\n";
        if (!reSolve3) {
            std::cout << "  -> OK: Executing swing is frozen\n";
        }
    }

    std::cout << "\n--- Proposed Logic ---\n";
    {
        SwingPlannerState state;
        double initTime = 0.25;

        // MPC iteration 1: t=0.25
        std::cout << "MPC iter 1 (t=" << initTime << "): ";
        bool reSolve1 = shouldReSolveProposed(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve1 ? "YES" : "NO (cached)") << "\n";
        if (reSolve1) {
            updateCacheProposed(state, swing1Start, swing1End, initTime, true);
            std::cout << "  -> QP solved, cache updated\n";
        }

        // MPC iteration 2: t=0.28
        initTime = 0.28;
        std::cout << "MPC iter 2 (t=" << initTime << "): ";
        bool reSolve2 = shouldReSolveProposed(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve2 ? "YES" : "NO (cached)") << "\n";
        if (!reSolve2) {
            std::cout << "  -> FIXED: Future swing is frozen after first solve\n";
        }

        // MPC iteration 3: t=0.32, swing is executing
        initTime = 0.32;
        std::cout << "MPC iter 3 (t=" << initTime << "): ";
        bool reSolve3 = shouldReSolveProposed(state, swing1Start, swing1End, initTime);
        std::cout << "reSolve=" << (reSolve3 ? "YES" : "NO (cached)") << "\n";
        if (!reSolve3) {
            std::cout << "  -> OK: Executing swing remains frozen\n";
        }
    }

    std::cout << "\n=== Summary ===\n";
    std::cout << "Current logic: Future swings are re-solved every MPC iteration,\n";
    std::cout << "               causing trajectory jitter as footholds change.\n";
    std::cout << "Proposed fix:  Freeze future swings after first solve, only\n";
    std::cout << "               re-solve when the swing actually starts executing.\n";

    return 0;
}
