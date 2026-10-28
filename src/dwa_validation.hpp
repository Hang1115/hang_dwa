#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "dwa_types.hpp"

namespace hang_dwa
{
    // 算法库内部共用的输入校验规则。
    namespace detail
    {
        inline bool isFinitePosition(double x, double y)
        {
            return std::isfinite(x) && std::isfinite(y);
        }

        inline bool isFinitePose(const Pose2D& pose)
        {
            return isFinitePosition(pose.x, pose.y) && std::isfinite(pose.yaw);
        }

        inline bool hasFinitePositions(const std::vector<Pose2D>& points)
        {
            return std::all_of(points.begin(), points.end(), [](const Pose2D& point) {
                return isFinitePosition(point.x, point.y);
            });
        }

        inline bool isRobotStateValid(const RobotState& state)
        {
            return isFinitePose(state.pose)
                && std::isfinite(state.velocity.vx)
                && std::isfinite(state.velocity.vy)
                && std::isfinite(state.velocity.wz)
                && (!state.has_last_command || (std::isfinite(state.last_command.vx)
                    && std::isfinite(state.last_command.vy) && std::isfinite(state.last_command.wz)));
        }

        inline bool isNonNegativeFinite(double value)
        {
            return std::isfinite(value) && value >= 0.0;
        }

        inline bool isPositiveFinite(double value)
        {
            return std::isfinite(value) && value > 0.0;
        }

        // 距离场允许 +inf 表示无障碍；NaN、负值和 -inf 均无效。
        inline bool isFieldDistanceValid(double distance)
        {
            return !std::isnan(distance) && distance >= 0.0;
        }

        // 地图结构校验仅检查几何和存储尺寸，不扫描格子数值。
        inline bool isMapGeometryValid(const MapGeometry& geometry)
        {
            return geometry.width > 0 && geometry.height > 0
                && isPositiveFinite(geometry.resolution)
                && isFinitePose(geometry.origin);
        }

        inline bool isGridDataSizeValid(const MapGeometry& geometry, std::size_t data_size)
        {
            if (geometry.width <= 0 || geometry.height <= 0)
            {
                return false;
            }

            const auto width = static_cast<std::size_t>(geometry.width);
            const auto height = static_cast<std::size_t>(geometry.height);
            return width <= std::numeric_limits<std::size_t>::max() / height
                && data_size == width * height;
        }

        inline bool isGridMapValid(const GridMap& map)
        {
            return isMapGeometryValid(map.map_geometry)
                && isGridDataSizeValid(map.map_geometry, map.data.size());
        }

        inline bool isDistanceFieldValid(const DistanceField& field)
        {
            return isMapGeometryValid(field.map_geometry)
                && isGridDataSizeValid(field.map_geometry, field.data.size());
        }

        // 配置与评价规则校验。
        inline bool isConfigValid(const DwaConfig& config)
        {
            const auto& limits = config.limits;
            return isNonNegativeFinite(limits.max_vx)
                && isNonNegativeFinite(limits.max_vy)
                && isNonNegativeFinite(limits.max_wz)
                && isNonNegativeFinite(limits.max_ax)
                && isNonNegativeFinite(limits.max_ay)
                && isNonNegativeFinite(limits.max_aw)
                && isPositiveFinite(limits.brake_ax)
                && isPositiveFinite(limits.brake_ay)
                && isPositiveFinite(limits.brake_aw)
                && isNonNegativeFinite(config.safety_margin)
                && isNonNegativeFinite(config.command_latency)
                && isPositiveFinite(config.goal_tolerance)
                && isNonNegativeFinite(config.stopped_speed)
                && isNonNegativeFinite(config.stopped_wz)
                && isPositiveFinite(config.approach_distance)
                && isPositiveFinite(config.goal_speed_gain)
                && isPositiveFinite(config.path_window_length)
                && isNonNegativeFinite(config.car_geometry.radius)
                && isPositiveFinite(config.control_dt)
                && isPositiveFinite(config.simulation_dt)
                && config.prediction_steps > 0
                && config.sample_count > 0
                && isPositiveFinite(config.influence_distance)
                && config.control_dt <= 1.0 && config.simulation_dt <= 1.0
                && config.command_latency <= 5.0
                && config.sample_count <= 31 && config.prediction_steps <= 2000
                && config.simulation_dt * config.prediction_steps <= 60.0
                && limits.max_vx / limits.brake_ax <= 60.0
                && limits.max_vy / limits.brake_ay <= 60.0
                && limits.max_wz / limits.brake_aw <= 60.0;
        }

        inline bool isCostTermValid(const CostTerm& term)
        {
            return term.func != nullptr && isNonNegativeFinite(term.weight);
        }

        inline bool areCostTermsValid(const std::vector<CostTerm>& terms)
        {
            return !terms.empty() && std::all_of(terms.begin(), terms.end(), isCostTermValid);
        }

        inline bool isPathCostInputValid(
            const std::vector<Pose2D>& trajectory,
            const std::vector<Pose2D>& reference_path)
        {
            return !trajectory.empty() && !reference_path.empty()
                && hasFinitePositions(trajectory) && hasFinitePositions(reference_path);
        }

        inline bool isObstacleCostInputValid(
            const std::vector<Pose2D>& trajectory,
            const DistanceField& field,
            const CarGeometry& geometry,
            double influence_distance)
        {
            return !trajectory.empty() && hasFinitePositions(trajectory)
                && isDistanceFieldValid(field)
                && isNonNegativeFinite(geometry.radius)
                && isPositiveFinite(influence_distance);
        }

        inline bool isTotalCostInputValid(
            const std::vector<Pose2D>& trajectory,
            const std::vector<CostTerm>& terms)
        {
            return !trajectory.empty() && areCostTermsValid(terms);
        }

        // 按固定顺序返回规划入口发现的首个输入错误。
        inline DwaStatus validateDwaInput(
            const RobotState& robot_state,
            const std::vector<Pose2D>& reference_path,
            const DistanceField& distance_field,
            const DwaConfig& config)
        {
            if (!isRobotStateValid(robot_state))
            {
                return DwaStatus::InvalidRobotState;
            }
            if (reference_path.empty())
            {
                return DwaStatus::EmptyReferencePath;
            }
            if (!hasFinitePositions(reference_path))
            {
                return DwaStatus::InvalidReferencePath;
            }
            if (!isDistanceFieldValid(distance_field))
            {
                return DwaStatus::InvalidDistanceField;
            }
            if (!isConfigValid(config) || config.path_start_index >= reference_path.size())
            {
                return DwaStatus::InvalidConfig;
            }
            if (!areCostTermsValid(config.cost_terms))
            {
                return DwaStatus::InvalidCostTerms;
            }
            return DwaStatus::Success;
        }
    }
}
