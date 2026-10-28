// 本文件实现由 Claude 补齐：坐标转换、距离场构建与查询。

#include "dwa_map.hpp"
#include "dwa_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace hang_dwa
{
    namespace
    {
        // 世界坐标 -> 地图局部连续坐标：减去 origin，再按 -origin.yaw 旋转。
        Pose2D worldToMapLocal(double world_x, double world_y, const MapGeometry& map)
        {
            const double dx = world_x - map.origin.x;
            const double dy = world_y - map.origin.y;
            const double cos_yaw = std::cos(map.origin.yaw);
            const double sin_yaw = std::sin(map.origin.yaw);
            return {cos_yaw * dx + sin_yaw * dy,
                    -sin_yaw * dx + cos_yaw * dy,
                    0.0};
        }

        // 未知格（-1）和占据值 >= occupied_threshold 的格子都算障碍。
        bool isBlockedCell(std::int8_t value, int occupied_threshold)
        {
            return value < 0 || static_cast<int>(value) >= occupied_threshold;
        }
    }

    bool worldToGrid(
        double world_x,
        double world_y,
        const MapGeometry& map,
        int& grid_x,
        int& grid_y)
    {
        if (!detail::isMapGeometryValid(map) || !detail::isFinitePosition(world_x, world_y))
        {
            return false;
        }

        const Pose2D local = worldToMapLocal(world_x, world_y, map);
        // 向下取整，避免地图原点外侧的负数被截断成第 0 格。
        const double column = std::floor(local.x / map.resolution);
        const double row = std::floor(local.y / map.resolution);
        if (!std::isfinite(column) || !std::isfinite(row)
            || column < 0.0 || column >= map.width
            || row < 0.0 || row >= map.height)
        {
            return false;
        }

        grid_x = static_cast<int>(column);
        grid_y = static_cast<int>(row);
        return true;
    }

    bool buildDistanceField(
        const GridMap& map,
        int occupied_threshold,
        DistanceField& output)
    {
        const MapGeometry& geometry = map.map_geometry;
        if (!detail::isGridMapValid(map) || occupied_threshold < 1 || occupied_threshold > 100
            || std::any_of(map.data.begin(), map.data.end(), [](int v) { return v < -1 || v > 100; }))
        {
            return false;
        }

        const std::size_t width = static_cast<std::size_t>(geometry.width);
        const std::size_t height = static_cast<std::size_t>(geometry.height);
        const std::size_t cell_count = width * height;

        const double kInfinity = std::numeric_limits<double>::infinity();
        std::vector<double> distance(cell_count, kInfinity);
        for (std::size_t index = 0; index < cell_count; ++index)
        {
            if (isBlockedCell(map.data[index], occupied_threshold))
            {
                distance[index] = 0.0;
            }
        }

        // 可分离的精确欧氏距离变换：先列后行，避免 chamfer 高估距离。
        auto transform = [](const std::vector<double>& input) {
            const int size = static_cast<int>(input.size());
            std::vector<double> result(size, std::numeric_limits<double>::infinity());
            std::vector<int> sites(size);
            std::vector<double> boundaries(size + 1);
            int last = -1;
            for (int q = 0; q < size; ++q)
            {
                if (!std::isfinite(input[q])) continue;
                double intersection = -std::numeric_limits<double>::infinity();
                while (last >= 0)
                {
                    const int v = sites[last];
                    intersection = ((input[q] + double(q) * q)
                        - (input[v] + double(v) * v)) / (2.0 * (q - v));
                    if (intersection > boundaries[last]) break;
                    --last;
                }
                ++last;
                sites[last] = q;
                boundaries[last] = last == 0
                    ? -std::numeric_limits<double>::infinity() : intersection;
                boundaries[last + 1] = std::numeric_limits<double>::infinity();
            }
            if (last < 0) return result;
            int site = 0;
            for (int q = 0; q < size; ++q)
            {
                while (site < last && boundaries[site + 1] < q) ++site;
                const double delta = q - sites[site];
                result[q] = delta * delta + input[sites[site]];
            }
            return result;
        };
        std::vector<double> line;
        line.resize(height);
        for (std::size_t col = 0; col < width; ++col)
        {
            for (std::size_t row = 0; row < height; ++row)
                line[row] = distance[row * width + col];
            const auto values = transform(line);
            for (std::size_t row = 0; row < height; ++row)
                distance[row * width + col] = values[row];
        }
        line.resize(width);
        for (std::size_t row = 0; row < height; ++row)
        {
            for (std::size_t col = 0; col < width; ++col)
                line[col] = distance[row * width + col];
            const auto values = transform(line);
            for (std::size_t col = 0; col < width; ++col)
                distance[row * width + col] = std::sqrt(values[col]) * geometry.resolution;
        }

        output.map_geometry = geometry;
        output.data = std::move(distance);
        return true;
    }

    bool queryDistance(
        double world_x,
        double world_y,
        const DistanceField& field,
        double& distance)
    {
        const MapGeometry& geometry = field.map_geometry;
        if (!detail::isDistanceFieldValid(field))
        {
            return false;
        }

        const std::size_t width = static_cast<std::size_t>(geometry.width);
        int grid_x = 0;
        int grid_y = 0;
        if (!worldToGrid(world_x, world_y, geometry, grid_x, grid_y))
        {
            return false;
        }

        distance = field.data[static_cast<std::size_t>(grid_y) * width
                              + static_cast<std::size_t>(grid_x)];
        return true;
    }
    bool queryClearance(
        double world_x, double world_y, const DistanceField& field,
        double radius, double& clearance)
    {
        double distance = 0.0;
        if (!detail::isNonNegativeFinite(radius)
            || !queryDistance(world_x, world_y, field, distance)
            || !detail::isFieldDistanceValid(distance)) return false;
        const auto& geometry = field.map_geometry;
        const Pose2D local = worldToMapLocal(world_x, world_y, geometry);
        const double resolution = geometry.resolution;
        const double center_x = (std::floor(local.x / resolution) + 0.5) * resolution;
        const double center_y = (std::floor(local.y / resolution) + 0.5) * resolution;
        // 距离场以格中心为基准，扣除查询偏移和障碍格外接圆半径。
        const double obstacle_distance = distance
            - std::hypot(local.x - center_x, local.y - center_y)
            - resolution * std::sqrt(0.5);
        const double boundary_distance = std::min({local.x, local.y,
            geometry.width * resolution - local.x,
            geometry.height * resolution - local.y});
        clearance = std::min(obstacle_distance, boundary_distance) - radius;
        return true;
    }

}
