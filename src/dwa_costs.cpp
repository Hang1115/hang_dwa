// DWA 评价函数实现。通过状态区分有效代价、不可行候选和评价错误。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "dwa_costs.hpp"
#include "dwa_map.hpp"
#include "dwa_types.hpp"
#include "dwa_validation.hpp"

namespace hang_dwa
{
    std::vector<CostTerm> makeDefaultCostTerms()
    {
        return {
            {calculatePathCost, defaults::path_cost_weight},
            {calculatePathProgressCost, defaults::path_progress_cost_weight},
            {calculateHeadingCost, defaults::heading_cost_weight},
            {calculateObstacleCost, defaults::obstacle_cost_weight},
            {calculateGoalCost, defaults::goal_cost_weight}
        };
    }

    namespace
    {
        constexpr double kInfinity = std::numeric_limits<double>::infinity();

        struct SegmentProjection
        {
            double x;
            double y;
            double ratio;
        };

        struct PathProjection
        {
            std::size_t segment{0};  // 线段终点的下标，从 1 开始。
            double ratio{0.0};
            double distance_squared{kInfinity};
        };

        // 点在有限线段上的投影。ratio 是段内比例 [0,1]，用于换算弧长。
        SegmentProjection projectPointToSegment(
            const Pose2D& point,
            const Pose2D& start,
            const Pose2D& end)
        {
            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double length_squared = dx * dx + dy * dy;
            if (!std::isfinite(length_squared))
            {
                return {kInfinity, kInfinity, 0.0};
            }

            double projection = 0.0;
            if (length_squared > 0.0)
            {
                projection = ((point.x - start.x) * dx + (point.y - start.y) * dy)
                    / length_squared;
                if (!std::isfinite(projection))
                {
                    return {kInfinity, kInfinity, 0.0};
                }
                projection = std::max(0.0, std::min(1.0, projection));
            }

            return {
                start.x + projection * dx,
                start.y + projection * dy,
                projection};
        }

        // 点到线段的平方距离。
        double distancePointToSegmentSquared(
            const Pose2D& point, const Pose2D& start, const Pose2D& end)
        {
            const auto projection = projectPointToSegment(point, start, end);

            const double error_x = point.x - projection.x;
            const double error_y = point.y - projection.y;
            return error_x * error_x + error_y * error_y;
        }

        // 点到折线的最近平方距离；空路径返回 +inf。
        double distancePointToPathSquared(
            const Pose2D& point, const std::vector<Pose2D>& path)
        {
            if (path.size() == 1)
            {
                return distancePointToSegmentSquared(point, path.front(), path.front());
            }

            double min_distance = kInfinity;
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const double distance_squared =
                    distancePointToSegmentSquared(point, path[i - 1], path[i]);
                if (!std::isfinite(distance_squared))
                {
                    return distance_squared;
                }
                min_distance = std::min(min_distance, distance_squared);
            }
            return min_distance;
        }

        // 折线累计弧长，单位米。
        std::vector<double> buildCumulativeLength(const std::vector<Pose2D>& path)
        {
            std::vector<double> cumulative(path.size(), 0.0);
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                cumulative[i] = cumulative[i - 1]
                    + std::hypot(path[i].x - path[i - 1].x,
                                 path[i].y - path[i - 1].y);
            }
            return cumulative;
        }

        // 把点投影到整条折线；失败时不修改 output。
        // 距离相同时保留较早的线段。
        bool projectPointOnPath(
            const Pose2D& point,
            const std::vector<Pose2D>& path,
            PathProjection& output,
            double min_arc = 0.0, double max_arc = kInfinity)
        {
            if (path.size() < 2)
            {
                return false;
            }

            PathProjection best_projection;
            double arc = 0.0;
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                auto projection = projectPointToSegment(point, path[i - 1], path[i]);
                const double length = std::hypot(path[i].x - path[i - 1].x,
                                                 path[i].y - path[i - 1].y);
                const double segment_start = arc;
                arc += length;
                if (arc < min_arc || segment_start > max_arc) continue;
                if (length > 0.0)
                {
                    const double low = std::max(0.0, (min_arc - segment_start) / length);
                    const double high = std::min(1.0, (max_arc - segment_start) / length);
                    projection.ratio = std::max(low, std::min(high, projection.ratio));
                    projection.x = path[i - 1].x + projection.ratio * (path[i].x - path[i - 1].x);
                    projection.y = path[i - 1].y + projection.ratio * (path[i].y - path[i - 1].y);
                }

                const double error_x = point.x - projection.x;
                const double error_y = point.y - projection.y;
                const double candidate = error_x * error_x + error_y * error_y;
                if (!std::isfinite(candidate))
                {
                    return false;
                }
                if (candidate < best_projection.distance_squared)
                {
                    best_projection = {i, projection.ratio, candidate};
                }
            }

            if (!std::isfinite(best_projection.distance_squared))
            {
                return false;
            }

            output = best_projection;
            return true;
        }

        // 顶点优先使用出发方向；末尾退回最后一段非零长度的前进方向。
        bool findPathHeading(
            const std::vector<Pose2D>& path,
            const PathProjection& projection,
            double& heading)
        {
            std::size_t segment = projection.segment;
            if (projection.ratio >= 1.0 && segment + 1 < path.size())
            {
                ++segment;
            }

            for (std::size_t i = segment; i < path.size(); ++i)
            {
                const double dx = path[i].x - path[i - 1].x;
                const double dy = path[i].y - path[i - 1].y;
                if (dx != 0.0 || dy != 0.0)
                {
                    heading = std::atan2(dy, dx);
                    return true;
                }
            }
            for (std::size_t i = segment; i > 0; --i)
            {
                const double dx = path[i].x - path[i - 1].x;
                const double dy = path[i].y - path[i - 1].y;
                if (dx != 0.0 || dy != 0.0)
                {
                    heading = std::atan2(dy, dx);
                    return true;
                }
            }
            return false;
        }

        // 将路径投影换算成距路径起点的弧长，单位米。
        double projectionToArcLength(
            const PathProjection& projection,
            const std::vector<double>& cumulative)
        {
            const std::size_t segment = projection.segment;
            return cumulative[segment - 1]
                + projection.ratio * (cumulative[segment] - cumulative[segment - 1]);
        }
    }

    // 以半格以内的间距覆盖整段轨迹，并用半采样间距膨胀圆形车体。
    // 硬碰撞与权重无关；平均接近代价让逐渐远离障碍的轨迹获益。
    CostResult calculateObstacleCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input)
    {
        if (!detail::isObstacleCostInputValid(
                trajectory, input.distance_field, input.car_geometry, input.influence_distance)
            || !detail::isNonNegativeFinite(input.safety_margin))
            return {CostStatus::InvalidInput};

        double sum = 0.0;
        double worst = 0.0;
        std::size_t count = 0;
        auto evaluate = [&](const Pose2D& point, double cover, bool score) -> CostStatus {
            int gx, gy;
            if (!worldToGrid(point.x, point.y, input.distance_field.map_geometry, gx, gy))
                return CostStatus::Infeasible;
            double clearance = 0.0;
            if (!queryClearance(point.x, point.y, input.distance_field,
                    input.car_geometry.radius + input.safety_margin + cover, clearance))
                return CostStatus::InvalidInput;
            if (clearance <= 0.0) return CostStatus::Infeasible;
            if (score)
            {
                const double proximity = std::max(0.0, 1.0 - clearance / input.influence_distance);
                const double cost = proximity * proximity;
                sum += cost;
                worst = std::max(worst, cost);
                ++count;
            }
            return CostStatus::Valid;
        };
        auto status = evaluate(trajectory.front(), 0.0, trajectory.size() == 1);
        if (status != CostStatus::Valid) return {status};
        const double step = input.distance_field.map_geometry.resolution * 0.5;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const auto& start = trajectory[i - 1];
            const auto& end = trajectory[i];
            const double length = std::hypot(end.x - start.x, end.y - start.y);
            if (!std::isfinite(length) || length / step > 1000000.0)
                return {CostStatus::NumericalError};
            const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
            const double cover = length / (2.0 * samples);
            // 两端都检查，使相邻圆的并集覆盖连接线段。
            status = evaluate(start, cover, false);
            if (status != CostStatus::Valid) return {status};
            for (int j = 1; j <= samples; ++j)
            {
                const double ratio = double(j) / samples;
                const Pose2D point{start.x + ratio * (end.x - start.x),
                                   start.y + ratio * (end.y - start.y), 0.0};
                status = evaluate(point, cover, true);
                if (status != CostStatus::Valid) return {status};
            }
        }
        return {CostStatus::Valid, 0.3 * worst + 0.7 * sum / std::max<std::size_t>(1, count)};
    }

    // 轨迹偏离参考折线的平均平方距离，单位 m²。
    CostResult calculatePathCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input)
    {
        if (!detail::isPathCostInputValid(trajectory, input.reference_path))
        {
            return {CostStatus::InvalidInput};
        }

        double squared_distance_sum = 0.0;
        for (const auto& pose : trajectory)
        {
            squared_distance_sum += distancePointToPathSquared(pose, input.reference_path);
            if (!std::isfinite(squared_distance_sum))
            {
                return {CostStatus::NumericalError};
            }
        }
        return {CostStatus::Valid, squared_distance_sum / static_cast<double>(trajectory.size())};
    }

    // 弧长进度：-(s_end - s_start)，单位米。前进为负，倒退为正。
    // 逐点投影的弧长变化受实际位移限制，防止折返/交叉路径跳段。
    CostResult calculatePathProgressCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input)
    {
        const auto& path = input.reference_path;

        if (!detail::isPathCostInputValid(trajectory, path))
        {
            return {CostStatus::InvalidInput};
        }
        if (path.size() == 1)
        {
            return {CostStatus::Valid, 0.0};
        }

        const std::vector<double> cumulative = buildCumulativeLength(path);
        const double total_length = cumulative.back();
        if (!std::isfinite(total_length))
        {
            return {CostStatus::NumericalError};
        }
        if (!(total_length > 0.0))
        {
            return {CostStatus::Valid, 0.0};
        }

        if (!detail::isPositiveFinite(input.progress_distance_scale)
            || !detail::isNonNegativeFinite(input.progress_weight_scale))
            return {CostStatus::InvalidInput};
        PathProjection projection;
        if (!projectPointOnPath(trajectory.front(), path, projection))
            return {CostStatus::NumericalError};
        const double start_arc = projectionToArcLength(projection, cumulative);
        double arc = start_arc;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            const double travel = std::hypot(trajectory[i].x - trajectory[i - 1].x,
                                             trajectory[i].y - trajectory[i - 1].y);
            if (!std::isfinite(travel) || !projectPointOnPath(trajectory[i], path, projection,
                    std::max(0.0, arc - travel), std::min(total_length, arc + travel)))
                return {CostStatus::NumericalError};
            arc = projectionToArcLength(projection, cumulative);
        }
        return {CostStatus::Valid,
            -(arc - start_arc) / input.progress_distance_scale * input.progress_weight_scale};
    }

    CostResult calculateHeadingCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input)
    {
        const auto& path = input.reference_path;
        if (!detail::isPathCostInputValid(trajectory, path)
            || !std::all_of(trajectory.begin(), trajectory.end(), detail::isFinitePose))
        {
            return {CostStatus::InvalidInput};
        }
        if (path.size() == 1)
        {
            return {CostStatus::Valid, 0.0};
        }

        double squared_error_sum = 0.0;
        const auto cumulative = buildCumulativeLength(path);
        double arc = 0.0;
        for (std::size_t i = 0; i < trajectory.size(); ++i)
        {
            const auto& pose = trajectory[i];
            const double travel = i == 0 ? kInfinity
                : std::hypot(pose.x - trajectory[i - 1].x, pose.y - trajectory[i - 1].y);
            PathProjection projection;
            if (!projectPointOnPath(pose, path, projection,
                    std::max(0.0, arc - travel), arc + travel))
            {
                return {CostStatus::NumericalError};
            }
            arc = projectionToArcLength(projection, cumulative);
            double heading = 0.0;
            if (!findPathHeading(path, projection, heading))
            {
                return {CostStatus::Valid, 0.0};
            }

            const double difference = pose.yaw - heading;
            const double error = std::atan2(std::sin(difference), std::cos(difference));
            squared_error_sum += error * error;
            if (!std::isfinite(squared_error_sum))
            {
                return {CostStatus::NumericalError};
            }
        }
        return {CostStatus::Valid, squared_error_sum / static_cast<double>(trajectory.size())};
    }

    CostResult calculateGoalCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input)
    {
        if (!detail::isPathCostInputValid(trajectory, input.reference_path)
            || !detail::isNonNegativeFinite(input.goal_cost_scale) || input.goal_cost_scale > 1.0)
            return {CostStatus::InvalidInput};
        const auto& goal = input.goal ? *input.goal : input.reference_path.back();
        if (!detail::isFinitePosition(goal.x, goal.y)) return {CostStatus::InvalidInput};
        const double dx = trajectory.back().x - goal.x;
        const double dy = trajectory.back().y - goal.y;
        double value = (dx * dx + dy * dy) * input.goal_cost_scale;
        if (input.local_goal)
        {
            const double local_dx = trajectory.back().x - input.local_goal->x;
            const double local_dy = trajectory.back().y - input.local_goal->y;
            value += 0.25 * (local_dx * local_dx + local_dy * local_dy)
                * (1.0 - input.goal_cost_scale);
        }
        if (!std::isfinite(value)) return {CostStatus::NumericalError};
        return {CostStatus::Valid, value};
    }

    // 加权总代价。先传播分项状态，再检查加权和累加的有限性。
    CostResult calculateTotalCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input,
        const std::vector<CostTerm>& cost_terms)
    {
        if (!detail::isTotalCostInputValid(trajectory, cost_terms))
        {
            return {CostStatus::InvalidInput};
        }

        double total_cost = 0.0;
        for (const auto& term : cost_terms)
        {
            const auto cost = term.func(trajectory, input);
            if (cost.status != CostStatus::Valid)
            {
                return cost;
            }
            if (!std::isfinite(cost.value))
            {
                return {CostStatus::NumericalError};
            }

            const double weighted_cost = term.weight * cost.value;
            total_cost += weighted_cost;
            if (!std::isfinite(weighted_cost) || !std::isfinite(total_cost))
            {
                return {CostStatus::NumericalError};
            }
        }
        return {CostStatus::Valid, total_cost};
    }
}
