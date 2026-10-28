#pragma once

#include <vector>

#include "dwa_types.hpp"

namespace hang_dwa
{
    // 返回固定的状态说明，由调用方决定是否记录或显示。
    const char* dwaStatusMessage(DwaStatus status);

    // 仅检查配置与评价表，不执行规划。
    bool isDwaConfigValid(const DwaConfig& config);

    // 位姿、路径和距离场使用同一规划坐标系，当前速度使用车体坐标系。
    // 路径至少一个点，仅使用 x/y；距离场几何和数据尺寸必须有效。
    // 距离场值由调用方保证非负且非 NaN，允许 +inf 表示无障碍。
    // 速度/加速度和半径非负，制动减速度为正；参数范围见 README。
    // 采样数、预测步数至少为 1；评价表非空，函数指针有效且权重非负且有限。
    // 输入不合法时返回首个检查失败的状态；未选出候选时返回 NoFeasibleTrajectory。
    // 评价错误时终止规划；GoalReached 表示位置与实际速度均满足容差，输出零速度。
    // 多段路径的 path_start_index 由调用方沿前进方向维护。
    DwaResult runDwa(
        const RobotState& robot_state,
        const std::vector<Pose2D>& reference_path,
        const DistanceField& field_map,
        const DwaConfig& config);
}
