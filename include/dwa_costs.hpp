// DWA 评价函数：路径偏离、沿路径进度、路径朝向、障碍接近和加权聚合。
// 返回评价状态及代价；仅 Valid 状态的 value 可用，代价越小越好。
#pragma once

#include <vector>

#include "dwa_types.hpp"

namespace hang_dwa
{
    // 碰撞或轨迹越界返回 Infeasible；查询到的距离允许为 +inf。
    CostResult calculateObstacleCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    CostResult calculatePathCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    CostResult calculatePathProgressCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    // 朝向参考路径前进切线的平均角度误差平方，轨迹 yaw 须有限。
    // 跳过重复路径点；整条路径无方向时返回零，不使用参考点的 yaw。
    CostResult calculateHeadingCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    // 评价预测末端与前方局部目标的误差，接近终点时切换到真实终点。
    CostResult calculateGoalCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    // 先校验评价表，再依次评价并加权；返回遇到的首个非 Valid 状态。
    // 零权重分项仍会执行；分项声明 Valid 时，其值必须有限。
    CostResult calculateTotalCost(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input,
        const std::vector<CostTerm>& cost_terms);
}
