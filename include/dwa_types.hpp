#pragma once

#include <cstdint>
#include <vector>
#include <limits>

#include "dwa_defaults.hpp"

namespace hang_dwa
{
    // ==================== 基础数据 ====================

    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double yaw{0.0};
    };

    struct Velocity2D
    {
        double vx{0.0};
        double vy{0.0};
        double wz{0.0};
    };

    // ==================== 机器人状态 ====================

    // 调用方每个控制周期更新；pose 使用规划坐标系，velocity 使用车体坐标系。
    struct RobotState
    {
        Pose2D pose;
        Velocity2D velocity;
        // 有执行延迟时，动态窗口沿上一条指令连续变化；velocity 仍为真实反馈。
        Velocity2D last_command{};
        bool has_last_command{false};
    };

    // ==================== 地图数据 ====================

    // resolution 单位为米/格，width、height 单位为格。
    struct MapGeometry
    {
        int width{0};
        int height{0};
        double resolution{0.0};
        Pose2D origin;
    };

    // 调用方提供的原始栅格地图，用于构建距离场。
    struct GridMap
    {
        MapGeometry map_geometry;
        // 与 OccupancyGrid 一致，按行存储：-1 未知，0 空闲，100 占据。
        std::vector<std::int8_t> data;
    };

    // 规划使用的障碍物距离场，可由 buildDistanceField 构建。
    struct DistanceField
    {
        MapGeometry map_geometry;
        // 当前格中心到最近障碍格中心的精确欧氏距离，单位米。
        std::vector<double> data;
    };

    // ==================== 车辆参数 ====================

    struct MotionLimits
    {
        double max_vx{defaults::max_vx};
        double max_vy{defaults::max_vy};
        double max_wz{defaults::max_wz};
        double max_ax{defaults::max_ax};
        double max_ay{defaults::max_ay};
        double max_aw{defaults::max_aw};
        double brake_ax{defaults::brake_ax};
        double brake_ay{defaults::brake_ay};
        double brake_aw{defaults::brake_aw};
    };

    // 保存车的几何信息。
    struct CarGeometry
    {
        double radius{defaults::car_radius};  // 圆形车体半径，单位米。
    };

    // ==================== 评价接口 ====================

    enum class CostStatus
    {
        Valid,           // 评价成功，代价值有效。
        Infeasible,      // 当前候选不可行，例如碰撞或轨迹越界。
        InvalidInput,    // 评价输入不符合约定。
        NumericalError   // 计算出现非有限结果或数值溢出。
    };

    // 仅在 status 为 Valid 时读取 value，有限代价越小越好。
    struct CostResult
    {
        CostStatus status{CostStatus::InvalidInput};
        double value{std::numeric_limits<double>::infinity()};
    };

    // 本次规划调用共享的评价输入，候选轨迹单独传入。
    // 路径、距离场和车体只引用调用方数据，评价表由聚合函数单独接收。
    struct CostInput
    {
        const std::vector<Pose2D>& reference_path;  // 参考折线
        const DistanceField& distance_field;       // 障碍距离场
        const CarGeometry& car_geometry;           // 车体几何
        const double influence_distance;          // 障碍影响距离，米
        double safety_margin{0.0};
        double progress_distance_scale{1.0};       // 将进度换算成无量纲代价
        double progress_weight_scale{1.0};         // 接近终点时衰减前进奖励
        const Pose2D* goal{nullptr};               // 局部路径之外的真实终点
        double goal_cost_scale{1.0};
        const Pose2D* local_goal{nullptr};          // 前方局部目标，帮助通过路径转角
    };

    // 内置和自定义评价函数使用的统一签名。
    using CostFunc = CostResult (*)(
        const std::vector<Pose2D>& trajectory,
        const CostInput& input);

    // 保存评价函数及其权重，由调用方填入规划配置。
    struct CostTerm
    {
        CostFunc func{nullptr};
        double weight{0.0};
    };

    // 创建包含内置评价函数及默认权重的独立评价表。
    std::vector<CostTerm> makeDefaultCostTerms();

    // ==================== 规划配置 ====================

    // 运动限制和车体尺寸需要由调用方按实际车辆设置。
    struct DwaConfig
    {
        MotionLimits limits;
        CarGeometry car_geometry;
        std::vector<CostTerm> cost_terms = makeDefaultCostTerms();

        double control_dt{defaults::control_dt};        // 动态窗口使用的控制周期，秒。
        double simulation_dt{defaults::simulation_dt};  // 预测轨迹的积分步长，秒。
        int prediction_steps{defaults::prediction_steps};
        int sample_count{defaults::sample_count};
        double influence_distance{defaults::influence_distance};  // 障碍影响距离，米。
        double safety_margin{defaults::safety_margin};
        double command_latency{defaults::command_latency};
        double goal_tolerance{defaults::goal_tolerance};
        double stopped_speed{defaults::stopped_speed};
        double stopped_wz{defaults::stopped_wz};
        double approach_distance{defaults::approach_distance};
        double goal_speed_gain{defaults::goal_speed_gain};
        double path_window_length{defaults::path_window_length};
        std::size_t path_start_index{0};  // 调用方维护已走过的路径进度
    };

    // ==================== 规划结果 ====================

    enum class DwaStatus
    {
        Success,                // 已选出可行轨迹。
        GoalReached,            // 位置和实际速度均满足到达条件，输出零速度。
        InvalidRobotState,      // 机器人位姿或速度包含非有限值。
        EmptyReferencePath,     // 参考路径为空。
        InvalidReferencePath,   // 参考路径的 x/y 包含非有限值。
        InvalidDistanceField,   // 距离场几何或数据尺寸无效。
        InvalidConfig,          // 运动限制、车体尺寸或规划参数无效。
        InvalidCostTerms,       // 评价表为空、函数指针为空或权重为负/非有限。
        InvalidCostInput,       // 评价过程中发现输入无效。
        CostNumericalError,     // 评价或加权聚合出现数值异常。
        NoFeasibleTrajectory    // 输入有效，但未选出可行轨迹。
    };

    // status 为 Success 时，最佳速度、轨迹和代价可用。
    struct DwaResult
    {
        DwaStatus status{DwaStatus::NoFeasibleTrajectory};
        Velocity2D best_velocity;
        std::vector<Pose2D> best_trajectory;
        double best_cost{std::numeric_limits<double>::infinity()};
    };

}
