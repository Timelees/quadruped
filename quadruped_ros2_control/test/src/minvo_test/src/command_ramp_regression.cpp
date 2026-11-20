#include <ocs2_quadruped_controller/control/CommandRamp.h>
#include <cmath>
#include <ocs2_quadruped_controller/control/TargetManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <stdexcept>
#include <iostream>
using ocs2::legged_robot::sampleCommandRamp;
void check(bool ok) { if (!ok) { throw std::runtime_error("Command braking regression failed"); } }
int main(int argc, char** argv) {
    if (argc == 3) {
        rclcpp::init(0, nullptr);
        CtrlInterfaces control;
        control.frequency_ = 500;
        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("command_target_regression");
        auto reference = std::make_shared<ocs2::ReferenceManager>();
        ocs2::legged_robot::TargetManager manager(control, node, reference, argv[1], argv[2]);
        ocs2::SystemObservation observation;
        observation.state = ocs2::vector_t::Zero(24);
        observation.input = ocs2::vector_t::Zero(24);
        observation.state[6] = 2.0;
        observation.state[8] = 0.28;
        for (int i = 0; i <= 200; ++i) {
            observation.time = 1.0 + i * 0.005;
            control.control_inputs_.ly = 0.5;
            control.control_inputs_.rx = -0.3;
            manager.update(observation);
        }
        control.control_inputs_.ly = 0;
        control.control_inputs_.rx = 0;
        observation.time += 0.005;
        manager.update(observation);
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        const auto& targets = reference->getTargetTrajectories();
        check(targets.stateTrajectory.front()[0] > 0.1);
        check(std::abs(targets.stateTrajectory.back()[0]) < 1e-12);
        check(targets.stateTrajectory.back()[6] > observation.state[6]);
        check(targets.stateTrajectory.back()[9] > observation.state[9]);
        for (size_t i = 1; i < targets.size(); ++i) {
            const double dt = targets.timeTrajectory[i] - targets.timeTrajectory[i - 1];
            const auto& a = targets.stateTrajectory[i - 1];
            const auto& b = targets.stateTrajectory[i];
            check(std::abs((b[6] - a[6]) - 0.5 * dt * (a[0] + b[0])) < 1e-10);
        }
        // A transient measured four-foot contact in planned trot must not
        // move the fixed world hold target to follow estimator/body drift.
        ocs2::ModeSchedule trot;
        trot.eventTimes = {observation.time + .2};
        trot.modeSequence = {9, 6};
        reference->setModeSchedule(trot);
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        for (int i = 0; i < 300; ++i) {
            observation.time += .005;
            observation.mode = 9;
            manager.update(observation);
        }
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        const double heldX = reference->getTargetTrajectories().stateTrajectory.back()[6];
        observation.state[6] += .05;
        observation.mode = 15;
        observation.time += .005;
        manager.update(observation);
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        check(std::abs(reference->getTargetTrajectories().stateTrajectory.back()[6] - heldX) < 1e-10);
        // Pure planned stance still aligns the hold with estimator startup.
        ocs2::ModeSchedule stand;
        stand.modeSequence = {15};
        reference->setModeSchedule(stand);
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        observation.time += .005;
        manager.update(observation);
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        check(std::abs(reference->getTargetTrajectories().stateTrajectory.back()[6] - observation.state[6]) < 1e-10);
        // A tilted body still requests horizontal lateral motion, rotated only
        // by yaw. Test the actual TargetManager rather than a copied formula.
        control.control_inputs_.lx = -0.8;
        observation.state[9] = 0.4;
        observation.state[10] = 0.2;
        observation.state[11] = -0.25;
        for (int i = 0; i < 300; ++i) {
            observation.time += .005;
            manager.update(observation);
        }
        reference->preSolverRun(observation.time, observation.time + 1.0, observation.state);
        const auto& lateral = reference->getTargetTrajectories().stateTrajectory.back();
        check(std::abs(lateral[2]) < 1e-12);
        check(lateral[1] > .1);
        check(std::abs(lateral[0] + std::tan(.4)*lateral[1]) < 1e-10);
        rclcpp::shutdown();
        std::cout << "Production target braking trajectory checks passed.\n";
    }

    for (double velocity : {-0.35, 0.35, -0.785, 0.785}) {
        const double acceleration = std::abs(velocity) > 0.4 ? 1.5 : 0.6;
        const double duration = std::abs(velocity) / acceleration;
        double previous = velocity;
        for (double t = 0.001; t < duration + 0.2; t += 0.001) {
            const auto sample = sampleCommandRamp(velocity, 0.0, acceleration, t);
            check(std::abs(sample.velocity - previous) <= acceleration * 0.001 + 1e-12);
            check(sample.velocity * velocity >= -1e-12);
            previous = sample.velocity;
        }
        const auto end = sampleCommandRamp(velocity, 0.0, acceleration, duration + 1);
        check(std::abs(end.velocity) < 1e-12);
        check(std::abs(end.displacement - 0.5 * velocity * duration) < 1e-12);
        // Receding-horizon integration preserves the same stopping endpoint.
        const auto first = sampleCommandRamp(velocity, 0.0, acceleration, duration * 0.4);
        const auto rest = sampleCommandRamp(first.velocity, 0.0, acceleration, duration);
        check(std::abs(first.displacement + rest.displacement - end.displacement) < 1e-12);
    }
    const auto reverse = sampleCommandRamp(0.2, -0.2, 0.6, 1.0);
    check(std::abs(reverse.velocity + 0.2) < 1e-12);
    std::cout << "Forward, reverse, and yaw braking checks passed.\n";
}
