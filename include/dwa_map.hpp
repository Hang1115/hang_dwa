// 地图工具：坐标转换、距离场构建和查询。不依赖 ROS。
#pragma once

#include "dwa_types.hpp"

namespace hang_dwa
{
    // 世界坐标 -> 格子列、行。点必须与 map.origin 同坐标系；
    // 越界、输入非有限或几何无效返回 false。
    bool worldToGrid(
        double world_x,
        double world_y,
        const MapGeometry& map,
        int& grid_x,
        int& grid_y);

    // 预计算到最近障碍格的距离场，单位米。未知格按障碍处理。
    // 每次地图更新调用一次；失败时不修改 output。
    bool buildDistanceField(
        const GridMap& map,
        int occupied_threshold,
        DistanceField& output);

    // 查询世界坐标到最近障碍的距离。地图内无障碍时返回 true，
    // distance 为 +inf；越界或距离场无效返回 false。
    bool queryDistance(
        double world_x,
        double world_y,
        const DistanceField& field,
        double& distance);

    // 圆形车体到障碍格边界及地图边界的保守净距；<=0 表示碰撞。
    bool queryClearance(
        double world_x, double world_y, const DistanceField& field,
        double radius, double& clearance);
}
