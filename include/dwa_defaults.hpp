#pragma once

namespace hang_dwa
{
    // 用于初始化配置对象，调用方可按实际车辆覆盖。
    namespace defaults
    {
        // 运动限制：线速度 m/s，角速度 rad/s，加速度 m/s²、rad/s²。
        constexpr double max_vx = 3.0;
        constexpr double max_vy = 0.1;
        constexpr double max_wz = 4.0;
        constexpr double max_ax = 1.0;
        constexpr double max_ay = 1.0;
        constexpr double max_aw = 1.0;
        // 可保证的制动减速度，与加速度上限分别设置。
        constexpr double brake_ax = 1.0;
        constexpr double brake_ay = 1.0;
        constexpr double brake_aw = 1.0;

        // 圆形车体半径，单位米。
        constexpr double car_radius = 0.6;

        // 动态窗口的控制周期和各速度轴的采样数量。
        constexpr double control_dt = 0.05;  // 秒
        constexpr int sample_count = 7;

        // 轨迹预测的积分步长和步数。
        constexpr double simulation_dt = 0.05;  // 秒
        constexpr int prediction_steps = 30;

        // 障碍评价的影响距离，单位米。
        constexpr double influence_distance = 0.8;
        constexpr double safety_margin = 0.05;
        constexpr double command_latency = 0.15;
        constexpr double goal_tolerance = 0.08;
        constexpr double stopped_speed = 0.03;
        constexpr double stopped_wz = 0.03;
        constexpr double approach_distance = 1.0;
        constexpr double goal_speed_gain = 1.0;
        constexpr double path_window_length = 3.0;

        // 内置评价函数的默认权重。
        constexpr double path_cost_weight = 2.0;
        constexpr double path_progress_cost_weight = 4.0;
        constexpr double heading_cost_weight = 0.5;
        constexpr double obstacle_cost_weight = 6.0;
        constexpr double goal_cost_weight = 4.0;
    }
}
