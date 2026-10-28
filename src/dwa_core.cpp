#include "dwa_core.hpp"
#include "dwa_costs.hpp"
#include "dwa_validation.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace hang_dwa
{
    namespace
    {
        // 仅供核心实现使用，表示当前控制周期内可采样的速度范围。
        struct DynamicWindow
        {
            Velocity2D min;
            Velocity2D max;
        };

        DynamicWindow calculateDynamicWindow(
            const Velocity2D& current_velocity,
            const MotionLimits& limits,
            double control_dt)
        {
            auto lower = [control_dt](double speed, double acceleration, double brake) {
                if (speed <= 0.0) return speed - acceleration * control_dt;
                const double stop_time = speed / brake;
                return stop_time >= control_dt ? speed - brake * control_dt
                    : -acceleration * (control_dt - stop_time);
            };
            DynamicWindow window;
            window.min = {
                std::max(lower(current_velocity.vx, limits.max_ax, limits.brake_ax), -limits.max_vx),
                std::max(lower(current_velocity.vy, limits.max_ay, limits.brake_ay), -limits.max_vy),
                std::max(lower(current_velocity.wz, limits.max_aw, limits.brake_aw), -limits.max_wz)};
            window.max = {
                std::min(-lower(-current_velocity.vx, limits.max_ax, limits.brake_ax), limits.max_vx),
                std::min(-lower(-current_velocity.vy, limits.max_ay, limits.brake_ay), limits.max_vy),
                std::min(-lower(-current_velocity.wz, limits.max_aw, limits.brake_aw), limits.max_wz)};
            return window;
        }

        // count 已由规划入口保证为正。
        std::vector<double> sampleAxis(double min, double max, int count)
        {
            if (min > max)
            {
                return {};
            }

            // 窗口只有一个可选速度，无需重复采样。
            if (min == max)
            {
                return {min};
            }

            if (count == 1)
            {
                return {min <= 0.0 && max >= 0.0 ? 0.0 : 0.5 * min + 0.5 * max};
            }

            const double step = (max - min) / (count - 1);
            std::vector<double> samples;
            samples.reserve(static_cast<std::size_t>(count));
            for (int i = 0; i < count; ++i)
            {
                samples.push_back(min + step * i);
            }
            // 即使均匀采样没命中零，也保留窗口内的停车候选。
            if (min < 0.0 && max > 0.0
                && std::none_of(samples.begin(), samples.end(), [](double v) { return v == 0.0; }))
                samples.push_back(0.0);
            return samples;
        }

        std::vector<Velocity2D> sampleVelocities(
            const DynamicWindow& window,
            int sample_count)
        {
            const auto vx_samples = sampleAxis(window.min.vx, window.max.vx, sample_count);
            const auto vy_samples = sampleAxis(window.min.vy, window.max.vy, sample_count);
            const auto wz_samples = sampleAxis(window.min.wz, window.max.wz, sample_count);
            std::vector<Velocity2D> velocities;
            velocities.reserve(vx_samples.size() * vy_samples.size() * wz_samples.size());
            for (double vx : vx_samples)
            {
                for (double vy : vy_samples)
                {
                    for (double wz : wz_samples)
                    {
                        velocities.push_back({vx, vy, wz});
                    }
                }
            }
            return velocities;
        }

        Pose2D predictNextPose(
            const Pose2D& current_pose,
            const Velocity2D& velocity,
            double simulation_dt)
        {
            const double turn = velocity.wz * simulation_dt;
            const double mid_yaw = current_pose.yaw + turn * 0.5;
            const double scale = std::abs(turn) < 1e-8 ? 1.0 : std::sin(turn * 0.5) / (turn * 0.5);
            const double cos_yaw = std::cos(mid_yaw) * scale;
            const double sin_yaw = std::sin(mid_yaw) * scale;
            return {
                current_pose.x + (velocity.vx * cos_yaw - velocity.vy * sin_yaw) * simulation_dt,
                current_pose.y + (velocity.vx * sin_yaw + velocity.vy * cos_yaw) * simulation_dt,
                current_pose.yaw + velocity.wz * simulation_dt};
        }

        // 时间步长和预测步数已由规划入口校验。
        std::vector<Pose2D> predictTrajectory(
            const Pose2D& initial_pose,
            const Velocity2D& velocity,
            double simulation_dt,
            int prediction_steps)
        {
            std::vector<Pose2D> trajectory;
            trajectory.reserve(static_cast<std::size_t>(prediction_steps) + 1);
            trajectory.push_back(initial_pose);
            for (int i = 0; i < prediction_steps; ++i)
            {
                trajectory.push_back(predictNextPose(trajectory.back(), velocity, simulation_dt));
            }
            return trajectory;
        }
        double towardsZero(double value, double delta)
        {
            return std::copysign(std::max(0.0, std::abs(value) - delta), value);
        }

        // 覆盖指令延迟、本周期速度过渡及持速，再检查立即制动能否停住。
        std::vector<Pose2D> predictSafetyTrajectory(
            const RobotState& state, const Velocity2D& command, const DwaConfig& config,
            double* curve_padding = nullptr)
        {
            std::vector<Pose2D> trajectory{state.pose};
            auto advance = [&](const Velocity2D& velocity, double duration) {
                const double speed = std::hypot(velocity.vx, velocity.vy);
                const double step = std::min(config.simulation_dt, 0.05 / std::max(1.0, speed));
                while (duration > 1e-9)
                {
                    const double dt = std::min(duration, step);
                    if (curve_padding)
                        *curve_padding = std::max(*curve_padding,
                            speed * std::abs(velocity.wz) * dt * dt / 8.0);
                    trajectory.push_back(predictNextPose(trajectory.back(), velocity, dt));
                    duration -= dt;
                }
            };
            advance(state.velocity, config.command_latency);
            const Velocity2D midpoint{
                0.5 * (state.velocity.vx + command.vx),
                0.5 * (state.velocity.vy + command.vy),
                0.5 * (state.velocity.wz + command.wz)};
            auto transition_time = [](double from, double to, double acceleration, double brake) {
                if (from * to < 0.0) return std::abs(from) / brake + std::abs(to) / std::max(1e-9, acceleration);
                return std::abs(to - from) / (std::abs(to) > std::abs(from)
                    ? std::max(1e-9, acceleration) : brake);
            };
            const double transition = std::max({config.control_dt,
                transition_time(state.velocity.vx, command.vx, config.limits.max_ax, config.limits.brake_ax),
                transition_time(state.velocity.vy, command.vy, config.limits.max_ay, config.limits.brake_ay),
                transition_time(state.velocity.wz, command.wz, config.limits.max_aw, config.limits.brake_aw)});
            advance(midpoint, transition);
            advance(command, config.control_dt);
            Velocity2D velocity = command;
            while (std::abs(velocity.vx) + std::abs(velocity.vy) + std::abs(velocity.wz) > 1e-9)
            {
                const double stop_time = std::max({std::abs(velocity.vx) / config.limits.brake_ax,
                    std::abs(velocity.vy) / config.limits.brake_ay,
                    std::abs(velocity.wz) / config.limits.brake_aw});
                const double dt = std::min(config.simulation_dt, stop_time);
                const Velocity2D next{
                    towardsZero(velocity.vx, config.limits.brake_ax * dt),
                    towardsZero(velocity.vy, config.limits.brake_ay * dt),
                    towardsZero(velocity.wz, config.limits.brake_aw * dt)};
                advance({0.5 * (velocity.vx + next.vx), 0.5 * (velocity.vy + next.vy),
                         0.5 * (velocity.wz + next.wz)}, dt);
                velocity = next;
            }
            return trajectory;
        }

        std::vector<Pose2D> selectLocalPath(
            const Pose2D& pose, const std::vector<Pose2D>& path, const DwaConfig& config)
        {
            if (path.size() == 1) return path;
            const std::size_t nearest = std::min(config.path_start_index, path.size() - 2);
            const auto& a = path[nearest];
            const auto& b = path[nearest + 1];
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double squared = dx * dx + dy * dy;
            const double ratio = squared > 0.0 ? std::max(0.0, std::min(1.0,
                ((pose.x - a.x) * dx + (pose.y - a.y) * dy) / squared)) : 0.0;
            const Pose2D start{a.x + ratio * dx, a.y + ratio * dy, 0.0};
            std::vector<Pose2D> local{start};
            const double window = std::max(config.path_window_length,
                std::hypot(config.limits.max_vx, config.limits.max_vy)
                    * config.simulation_dt * config.prediction_steps);
            double length = 0.0;
            for (std::size_t i = nearest + 1; i < path.size(); ++i)
            {
                length += std::hypot(path[i].x - local.back().x, path[i].y - local.back().y);
                local.push_back(path[i]);
                if (length >= window) break;
            }
            return local;
        }

    }

    bool isDwaConfigValid(const DwaConfig& config)
    {
        return detail::isConfigValid(config) && detail::areCostTermsValid(config.cost_terms);
    }

    const char* dwaStatusMessage(DwaStatus status)
    {
        switch (status)
        {
        case DwaStatus::Success:
            return "Success";
        case DwaStatus::GoalReached:
            return "Goal reached and robot stopped";
        case DwaStatus::InvalidRobotState:
            return "Robot pose or velocity contains a non-finite value";
        case DwaStatus::EmptyReferencePath:
            return "Reference path is empty";
        case DwaStatus::InvalidReferencePath:
            return "Reference path contains a non-finite position";
        case DwaStatus::InvalidDistanceField:
            return "Distance field geometry or data size is invalid";
        case DwaStatus::InvalidConfig:
            return "DWA configuration is invalid";
        case DwaStatus::InvalidCostTerms:
            return "Cost terms are empty or contain a null function or invalid weight";
        case DwaStatus::InvalidCostInput:
            return "Cost evaluation input is invalid";
        case DwaStatus::CostNumericalError:
            return "Cost evaluation or aggregation produced a numerical error";
        case DwaStatus::NoFeasibleTrajectory:
            return "No feasible trajectory was selected";
        }
        return "Unknown DWA status";
    }

    DwaResult runDwa(
        const RobotState& robot_state,
        const std::vector<Pose2D>& reference_path,
        const DistanceField& field_map,
        const DwaConfig& config)
    {
        DwaResult result;
        const auto status = detail::validateDwaInput(robot_state, reference_path, field_map, config);
        if (status != DwaStatus::Success)
        {
            result.status = status;
            return result;
        }

        const auto local_path = selectLocalPath(robot_state.pose, reference_path, config);
        const auto& goal = reference_path.back();
        const double goal_distance = std::hypot(robot_state.pose.x - goal.x, robot_state.pose.y - goal.y);
        const bool goal_visible = local_path.back().x == goal.x && local_path.back().y == goal.y;
        double remaining_distance = 0.0;
        Pose2D previous = local_path.front();
        for (std::size_t i = config.path_start_index + 1; i < reference_path.size(); ++i)
        {
            remaining_distance += std::hypot(reference_path[i].x - previous.x,
                                              reference_path[i].y - previous.y);
            previous = reference_path[i];
        }
        const double approach_distance = std::max(goal_distance, remaining_distance);
        const double goal_scale = goal_visible
            ? std::max(0.0, 1.0 - approach_distance / config.approach_distance) : 0.0;
        const auto tracking_window = calculateDynamicWindow(robot_state.velocity, config.limits,
            config.command_latency + 2.0 * config.control_dt);
        const auto& last = robot_state.last_command;
        const bool command_tracks_feedback = robot_state.has_last_command
            && last.vx >= tracking_window.min.vx - 1e-9 && last.vx <= tracking_window.max.vx + 1e-9
            && last.vy >= tracking_window.min.vy - 1e-9 && last.vy <= tracking_window.max.vy + 1e-9
            && last.wz >= tracking_window.min.wz - 1e-9 && last.wz <= tracking_window.max.wz + 1e-9;
        const auto& window_velocity = command_tracks_feedback ? last : robot_state.velocity;
        const auto window = calculateDynamicWindow(window_velocity, config.limits, config.control_dt);
        const auto velocities = sampleVelocities(window, config.sample_count);
        const double travel_scale = std::max(1.0, std::hypot(config.limits.max_vx, config.limits.max_vy)
            * config.simulation_dt * config.prediction_steps);
        double lookahead = std::max(1.0, std::hypot(robot_state.velocity.vx, robot_state.velocity.vy)
            * config.simulation_dt * config.prediction_steps);
        Pose2D local_goal = local_path.back();
        for (std::size_t i = 1; i < local_path.size(); ++i)
        {
            const auto& a = local_path[i - 1];
            const auto& b = local_path[i];
            const double length = std::hypot(b.x - a.x, b.y - a.y);
            if (length >= lookahead && length > 0.0)
            {
                local_goal = {a.x + (b.x - a.x) * lookahead / length,
                              a.y + (b.y - a.y) * lookahead / length, 0.0};
                break;
            }
            lookahead -= length;
        }
        const CostInput cost_input{local_path, field_map, config.car_geometry,
            config.influence_distance, config.safety_margin, travel_scale,
            1.0 - goal_scale, &goal, goal_scale, &local_goal};

        // 到达必须同时满足位置、线速度及角速度，并通过当前位置碰撞检查。
        if (goal_visible && approach_distance <= config.goal_tolerance
            && std::hypot(robot_state.velocity.vx, robot_state.velocity.vy) <= config.stopped_speed
            && std::abs(robot_state.velocity.wz) <= config.stopped_wz
            && std::abs(robot_state.velocity.vx) <= config.limits.brake_ax * config.control_dt
            && std::abs(robot_state.velocity.vy) <= config.limits.brake_ay * config.control_dt
            && std::abs(robot_state.velocity.wz) <= config.limits.brake_aw * config.control_dt
            && window.min.vx <= 0.0 && window.max.vx >= 0.0
            && window.min.vy <= 0.0 && window.max.vy >= 0.0
            && window.min.wz <= 0.0 && window.max.wz >= 0.0
            && calculateObstacleCost(predictSafetyTrajectory(robot_state, {}, config), cost_input).status
                == CostStatus::Valid)
        {
            result.status = DwaStatus::GoalReached;
            result.best_trajectory = {robot_state.pose};
            result.best_cost = 0.0;
            return result;
        }
        auto nearest_zero = [](double low, double high) {
            return low > 0.0 ? low : (high < 0.0 ? high : 0.0);
        };
        const double minimum_speed = std::hypot(nearest_zero(window.min.vx, window.max.vx),
                                                 nearest_zero(window.min.vy, window.max.vy));
        const double minimum_wz = std::abs(nearest_zero(window.min.wz, window.max.wz));
        const double distance = std::max(0.0, approach_distance - config.goal_tolerance * 0.5);
        const double brake = std::min(config.limits.brake_ax, config.limits.brake_ay);
        const double delay = config.command_latency + config.control_dt;
        const double safe_speed = std::sqrt(brake * brake * delay * delay + 2.0 * brake * distance)
            - brake * delay;
        const double speed_limit = std::max(minimum_speed,
            approach_distance <= config.goal_tolerance ? 0.0
                : std::min(config.goal_speed_gain * distance, safe_speed));
        const double wz_limit = goal_visible && approach_distance < config.approach_distance
            ? std::max(minimum_wz, config.goal_speed_gain * distance)
            : std::numeric_limits<double>::infinity();

        for (const auto& velocity : velocities)
        {
            if (std::hypot(velocity.vx, velocity.vy) > speed_limit + 1e-9
                || std::abs(velocity.wz) > wz_limit + 1e-9) continue;
            double curve_padding = 0.0;
            const auto safety_trajectory = predictSafetyTrajectory(robot_state, velocity, config, &curve_padding);
            const auto trajectory = predictTrajectory(
                robot_state.pose, velocity, config.simulation_dt, config.prediction_steps);
            curve_padding = std::max(curve_padding,
                std::hypot(velocity.vx, velocity.vy) * std::abs(velocity.wz)
                    * config.simulation_dt * config.simulation_dt / 8.0);
            const CarGeometry safety_geometry{config.car_geometry.radius + curve_padding};
            const CostInput safety_input{local_path, field_map, safety_geometry,
                config.influence_distance, config.safety_margin};
            auto safety = calculateObstacleCost(safety_trajectory, safety_input);
            if (safety.status == CostStatus::Valid)
                safety = calculateObstacleCost(trajectory, safety_input);
            if (safety.status == CostStatus::Infeasible) continue;
            if (safety.status != CostStatus::Valid)
            {
                result.status = safety.status == CostStatus::InvalidInput
                    ? DwaStatus::InvalidCostInput : DwaStatus::CostNumericalError;
                return result;
            }
            const auto candidate_cost = calculateTotalCost(trajectory, cost_input, config.cost_terms);
            if (candidate_cost.status == CostStatus::Infeasible)
            {
                continue;
            }
            if (candidate_cost.status != CostStatus::Valid)
            {
                DwaResult failure;
                failure.status = candidate_cost.status == CostStatus::InvalidInput
                    ? DwaStatus::InvalidCostInput : DwaStatus::CostNumericalError;
                return failure;
            }
            if (candidate_cost.value < result.best_cost)
            {
                result.best_cost = candidate_cost.value;
                result.best_trajectory = trajectory;
                result.best_velocity = velocity;
                result.status = DwaStatus::Success;
            }
        }
        return result;
    }
}
