#include "trajectory_planner.hpp"
#include "logger.hpp"
#include <urdf_parser/urdf_parser.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

struct URDFMotionPlanner::Impl {
    ompl::geometric::SimpleSetupPtr setup;
    ompl::base::StateSpacePtr state_space;
    unsigned int num_joints = 0;

    bool setupPlanner(const std::vector<double>& lower_bounds,
                      const std::vector<double>& upper_bounds) {
        num_joints = lower_bounds.size();
        if (num_joints == 0) return false;

        state_space = std::make_shared<ompl::base::RealVectorStateSpace>(num_joints);
        ompl::base::RealVectorBounds bounds(num_joints);
        for (unsigned i = 0; i < num_joints; ++i) {
            bounds.setLow(i, lower_bounds[i]);
            bounds.setHigh(i, upper_bounds[i]);
        }
        state_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

        setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space);

        auto rrt_connect = std::make_shared<ompl::geometric::RRTConnect>(setup->getSpaceInformation());
        rrt_connect->setRange(0.3);
        setup->setPlanner(rrt_connect);

        setup->getSpaceInformation()->setStateValidityChecker(
            [](const ompl::base::State*) { return true; });

        setup->setup();
        return true;
    }

    bool plan(const std::vector<double>& start,
              const std::vector<double>& goal,
              std::vector<std::vector<double>>& waypoints,
              double planning_time_sec) {
        ompl::base::ScopedState<> start_state(state_space);
        ompl::base::ScopedState<> goal_state(state_space);
        for (unsigned i = 0; i < num_joints; ++i) {
            start_state[i] = start[i];
            goal_state[i] = goal[i];
        }

        setup->setStartState(start_state);
        setup->setGoalState(goal_state);
        setup->clear();

        auto solved = setup->solve(planning_time_sec);
        if (!solved) {
            LOG_WARN("OMPL RRTConnect: no solution in {:.1f}s", planning_time_sec);
            return false;
        }

        auto path = setup->getSolutionPath();
        path.interpolate();
        auto& states = path.getStates();

        LOG_INFO("OMPL RRTConnect: solution found ({:.3f}s, {} states)",
                 setup->getLastPlanComputationTime(), states.size());

        waypoints.clear();
        waypoints.reserve(states.size());
        for (auto* s : states) {
            auto* rs = s->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> wp(num_joints);
            for (unsigned i = 0; i < num_joints; ++i) {
                wp[i] = (*rs)[i];
            }
            waypoints.push_back(std::move(wp));
        }
        return true;
    }
};

URDFMotionPlanner::URDFMotionPlanner()
    : impl_(std::make_unique<Impl>()) {}

URDFMotionPlanner::~URDFMotionPlanner() = default;

bool URDFMotionPlanner::LoadURDF(const std::string& urdf_path) {
    auto model = urdf::parseURDFFile(urdf_path);
    if (!model) {
        LOG_ERROR("Failed to parse URDF: {}", urdf_path);
        return false;
    }

    joint_names_.clear();
    lower_bounds_.clear();
    upper_bounds_.clear();

    for (const auto& [name, joint] : model->joints_) {
        if (joint->type == urdf::Joint::REVOLUTE ||
            joint->type == urdf::Joint::CONTINUOUS ||
            joint->type == urdf::Joint::PRISMATIC) {
            joint_names_.push_back(name);
            double lo = -M_PI, hi = M_PI;
            if (joint->limits) {
                lo = joint->limits->lower;
                hi = joint->limits->upper;
            }
            if (std::abs(hi - lo) < 1e-6) {
                lo = -M_PI;
                hi = M_PI;
            }
            lower_bounds_.push_back(lo);
            upper_bounds_.push_back(hi);
        }
    }

    if (joint_names_.empty()) {
        LOG_ERROR("No movable joints found in URDF");
        return false;
    }

    LOG_INFO("URDFMotionPlanner: {} movable joints", joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        LOG_DEBUG("  joint[{}] {}: [{:.3f}, {:.3f}]",
                  i, joint_names_[i], lower_bounds_[i], upper_bounds_[i]);
    }

    return impl_->setupPlanner(lower_bounds_, upper_bounds_);
}

JointTrajectory URDFMotionPlanner::PlanToTarget(
    const std::vector<double>& current_positions,
    const std::vector<double>& target_positions,
    double planning_time_sec) {

    JointTrajectory result;

    if (current_positions.size() != num_joints() ||
        target_positions.size() != num_joints()) {
        LOG_ERROR("Joint count mismatch: current={}, target={}, expected={}",
                  current_positions.size(), target_positions.size(), num_joints());
        return result;
    }

    std::vector<std::vector<double>> waypoints;
    bool ok = impl_->plan(current_positions, target_positions, waypoints, planning_time_sec);
    if (!ok || waypoints.size() < 2) {
        LOG_WARN("Planning failed, using direct target");
        JointTrajectoryPoint pt;
        pt.positions = target_positions;
        pt.time_from_start = 1.0;
        result.points.push_back(std::move(pt));
        return result;
    }

    ruckig::Ruckig<0> ruckig(num_joints());

    auto wp_to_input = [&](ruckig::InputParameter<0>& input,
                           const std::vector<double>& current,
                           const std::vector<double>& target) {
        for (size_t i = 0; i < num_joints(); ++i) {
            input.current_position[i] = current[i];
            input.current_velocity[i] = 0.0;
            input.current_acceleration[i] = 0.0;
            input.target_position[i] = target[i];
            input.target_velocity[i] = 0.0;
            input.target_acceleration[i] = 0.0;
            input.max_velocity[i] = 1.0;
            input.max_acceleration[i] = 2.0;
            input.max_jerk[i] = 5.0;
        }
        input.synchronization = ruckig::Synchronization::Time;
    };

    double total_time_offset = 0.0;

    for (size_t w = 0; w + 1 < waypoints.size(); ++w) {
        ruckig::InputParameter<0> input(num_joints());
        wp_to_input(input, waypoints[w], waypoints[w + 1]);

        ruckig::Trajectory<0> trajectory(num_joints());
        auto status = ruckig.calculate(input, trajectory);

        if (status != ruckig::Result::Working) {
            LOG_WARN("Ruckig failed at segment {}", w);
            JointTrajectoryPoint pt;
            pt.positions = waypoints[w + 1];
            pt.time_from_start = total_time_offset + 0.1;
            result.points.push_back(std::move(pt));
            total_time_offset += 0.1;
            continue;
        }

        double dur = trajectory.get_duration();
        double dt = 0.02;
        for (double t = 0.0; t <= dur; t += dt) {
            JointTrajectoryPoint pt;
            pt.positions.resize(num_joints());
            pt.velocities.resize(num_joints());
            pt.accelerations.resize(num_joints());
            trajectory.at_time(t, pt.positions, pt.velocities, pt.accelerations);
            pt.time_from_start = total_time_offset + t;
            result.points.push_back(std::move(pt));
        }
        total_time_offset += dur;
    }

    LOG_INFO("URDFMotionPlanner: {} trajectory pts across {} OMPL waypoints, duration {:.2f}s",
             result.points.size(), waypoints.size(), total_time_offset);
    return result;
}
