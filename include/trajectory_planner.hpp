#pragma once

#include <vector>
#include <string>
#include <memory>
#include <chrono>

struct JointTrajectoryPoint {
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
    double time_from_start = 0.0;
};

struct JointTrajectory {
    std::vector<JointTrajectoryPoint> points;
};

class URDFMotionPlanner {
public:
    URDFMotionPlanner();
    ~URDFMotionPlanner();

    bool LoadURDF(const std::string& urdf_path);

    JointTrajectory PlanToTarget(
        const std::vector<double>& current_positions,
        const std::vector<double>& target_positions,
        double planning_time_sec = 5.0);

    std::vector<std::string> getJointNames() const { return joint_names_; }
    std::vector<double> getLowerBounds() const { return lower_bounds_; }
    std::vector<double> getUpperBounds() const { return upper_bounds_; }
    size_t num_joints() const { return joint_names_.size(); }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::vector<std::string> joint_names_;
    std::vector<double> lower_bounds_;
    std::vector<double> upper_bounds_;
};

struct TrajectoryExecutor {
    JointTrajectory trajectory;
    std::chrono::steady_clock::time_point start_time;
    size_t current_index = 0;
    bool active = false;

    void Start(const JointTrajectory& traj) {
        trajectory = traj;
        start_time = std::chrono::steady_clock::now();
        current_index = 0;
        active = true;
    }

    void Stop() {
        active = false;
        current_index = 0;
        trajectory = {};
    }

    bool GetCurrentPosition(std::vector<double>& positions) {
        if (!active || trajectory.points.empty()) {
            return false;
        }
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        while (current_index < trajectory.points.size() &&
               trajectory.points[current_index].time_from_start <= elapsed) {
            ++current_index;
        }
        if (current_index >= trajectory.points.size()) {
            positions = trajectory.points.back().positions;
            active = false;
            return true;
        }
        positions = trajectory.points[current_index].positions;
        return true;
    }
};
