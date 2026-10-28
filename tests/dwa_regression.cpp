#include "dwa_core.hpp"
#include "dwa_map.hpp"
#include "dwa_costs.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hang_dwa;
namespace
{
    void require(bool condition, const std::string& message)
    {
        if (!condition) throw std::runtime_error(message);
    }
    GridMap freeMap()
    {
        return {{160, 120, 0.1, {-4.0, -4.0, 0.0}}, std::vector<std::int8_t>(160 * 120, 0)};
    }
    void block(GridMap& map, double x, double y)
    {
        int gx, gy;
        require(worldToGrid(x, y, map.map_geometry, gx, gy), "fixture obstacle outside map");
        map.data[gy * map.map_geometry.width + gx] = 100;
    }
    DistanceField fieldOf(const GridMap& map)
    {
        DistanceField field;
        require(buildDistanceField(map, 65, field), "distance field build failed");
        return field;
    }
    CostResult zeroCost(const std::vector<Pose2D>&, const CostInput&)
    {
        return {CostStatus::Valid, 0.0};
    }
    void distanceAndSweep()
    {
        auto map = freeMap();
        block(map, 1.05, 1.05);
        block(map, -0.75, 2.15);
        const auto field = fieldOf(map);
        for (int row = 0; row < map.map_geometry.height; ++row)
            for (int col = 0; col < map.map_geometry.width; ++col)
            {
                double expected = INFINITY;
                for (int r = 0; r < map.map_geometry.height; ++r)
                    for (int c = 0; c < map.map_geometry.width; ++c)
                        if (map.data[r * map.map_geometry.width + c] == 100)
                            expected = std::min(expected, std::hypot(double(row - r), double(col - c)) * 0.1);
                require(std::abs(field.data[row * map.map_geometry.width + col] - expected) < 1e-10,
                        "EDT differs from brute-force Euclidean distance");
            }
        const std::vector<Pose2D> path{{0, 0, 0}, {3, 0, 0}};
        CarGeometry car{0.02};
        CostInput input{path, field, car, 0.8};
        require(calculateObstacleCost({{0.05, 1.05, 0}, {2.05, 1.05, 0}}, input).status
                    == CostStatus::Infeasible, "thin obstacle skipped between endpoints");
        auto empty = fieldOf(freeMap());
        CarGeometry big_car{0.3};
        CostInput boundary{path, empty, big_car, 0.8};
        require(calculateObstacleCost({{-3.9, 0, 0}}, boundary).status == CostStatus::Infeasible,
                "footprint allowed across map boundary");
        map.map_geometry.origin.yaw = 0.6;
        const auto rotated = fieldOf(map);
        double clearance;
        const double x = -4.0 + 0.1 * std::cos(0.6) - 2.0 * std::sin(0.6);
        const double y = -4.0 + 0.1 * std::sin(0.6) + 2.0 * std::cos(0.6);
        require(queryClearance(x, y, rotated, 0.3, clearance) && clearance < 0,
                "rotated map footprint boundary not checked");
        auto unknown_map = freeMap();
        unknown_map.data[40 * 160 + 40] = -1;
        require(fieldOf(unknown_map).data[40 * 160 + 40] == 0.0, "unknown cell not blocked");
    }
    void departureAndProgress()
    {
        auto map = freeMap();
        block(map, 0.05, -0.55);
        const auto field = fieldOf(map);
        const std::vector<Pose2D> path{{0.05, 0.05, 0}, {2, 0.05, 0}};
        CarGeometry car{0.2};
        const CostInput input{path, field, car, 0.8};
        const auto stay = calculateObstacleCost({{0.05, 0.05, 0}, {0.05, 0.05, 0}, {0.05, 0.05, 0}}, input);
        const auto leave = calculateObstacleCost({{0.05, 0.05, 0}, {0.05, 0.35, 0}, {0.05, 0.65, 0}}, input);
        require(stay.status == CostStatus::Valid && leave.status == CostStatus::Valid
                    && leave.value < stay.value, "moving away from obstacle has no benefit");
        const std::vector<Pose2D> folded{{0,0,0}, {1,0,0}, {1,0.1,0}, {0,0.1,0}};
        const CostInput folded_input{folded, field, car, 0.8};
        const std::vector<Pose2D> step{{0.2,0.049,0}, {0.2,0.051,0}};
        const auto cost = calculatePathProgressCost(step, folded_input);
        require(cost.status == CostStatus::Valid && std::abs(cost.value) <= 0.002 + 1e-9,
                "projection jumped to a later folded segment");
    }
    void mandatorySafety()
    {
        auto map = freeMap();
        for (double y = -3.0; y < 6.0; y += 0.05) block(map, 0.85, y);
        const auto field = fieldOf(map);
        DwaConfig config;
        config.car_geometry.radius = 0.1;
        config.safety_margin = 0.0;
        config.simulation_dt = 0.05;
        config.prediction_steps = 2;
        config.command_latency = 0.3;
        config.limits.max_vy = 0;
        config.limits.max_wz = 0;
        config.cost_terms = {{zeroCost, 1.0}};
        RobotState state{{0.05, 0.05, 0}, {1.0, 0, 0}};
        auto result = runDwa(state, {{0.05,0.05,0}, {4,0.05,0}}, field, config);
        require(result.status == DwaStatus::NoFeasibleTrajectory,
                "latency/braking collision accepted when obstacle critic omitted");
        config.command_latency = 0.0;
        config.limits.brake_ax = 0.2;
        result = runDwa(state, {{0.05,0.05,0}, {4,0.05,0}}, field, config);
        require(result.status == DwaStatus::NoFeasibleTrajectory, "braking tail collision accepted");
        config.cost_terms = {{calculateObstacleCost, 0.0}};
        state.pose = {0.85, 0.05, 0};
        result = runDwa(state, {{0.05,0.05,0}, {4,0.05,0}}, field, config);
        require(result.status == DwaStatus::NoFeasibleTrajectory, "zero obstacle weight disabled safety");
        config.limits.brake_ax = 0;
        require(!isDwaConfigValid(config), "zero braking deceleration accepted");
    }
    void advance(RobotState& state, double dt)
    {
        const double turn = state.velocity.wz * dt;
        const double scale = std::abs(turn) < 1e-9 ? 1.0 : std::sin(turn / 2.0) / (turn / 2.0);
        const double yaw = state.pose.yaw + turn / 2.0;
        state.pose.x += (state.velocity.vx * std::cos(yaw) - state.velocity.vy * std::sin(yaw)) * dt * scale;
        state.pose.y += (state.velocity.vx * std::sin(yaw) + state.velocity.vy * std::cos(yaw)) * dt * scale;
        state.pose.yaw += turn;
    }
    void closedLoop(double goal_x, double initial_speed, int delayed_steps = 0)
    {
        const auto field = fieldOf(freeMap());
        DwaConfig config;
        config.limits.max_vy = 0;
        config.limits.max_wz = 0;
        RobotState state{{0,0,0}, {initial_speed,0,0}};
        std::vector<Pose2D> path{{0,0,0}, {goal_x,0,0}};
        if (goal_x < 0) path = {{goal_x - 2,0,0}, {goal_x,0,0}};
        std::deque<Velocity2D> pending(delayed_steps, state.velocity);
        double overshoot = 0.0;
        double slowest_ms = 0.0;
        bool reached = false;
        for (int i = 0; i < 1000; ++i)
        {
            const auto begin = std::chrono::steady_clock::now();
            const auto result = runDwa(state, path, field, config);
            slowest_ms = std::max(slowest_ms, std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count());
            if (result.status == DwaStatus::GoalReached)
            {
                reached = true;
                require(std::abs(state.pose.x - goal_x) <= config.goal_tolerance, "goal position tolerance violated");
                require(std::abs(state.velocity.vx) <= config.stopped_speed, "goal reached while moving");
                require(result.best_velocity.vx == 0, "goal result not zero");
                break;
            }
            if (result.status != DwaStatus::Success)
                std::cerr << "failure step=" << i << " x=" << state.pose.x << " speed=" << state.velocity.vx
                          << " delay=" << delayed_steps << '\n';
            require(result.status == DwaStatus::Success, std::string("closed loop failed: ") + dwaStatusMessage(result.status));
            const double change = result.best_velocity.vx
                - (state.has_last_command ? state.last_command.vx : state.velocity.vx);
            require(std::abs(change) <= config.limits.max_ax * config.control_dt + 1e-8,
                    "selected speed outside dynamic window");
            state.last_command = result.best_velocity;
            state.has_last_command = true;
            pending.push_back(result.best_velocity);
            state.velocity = pending.front();
            pending.pop_front();
            advance(state, config.control_dt);
            if (goal_x > 0) overshoot = std::max(overshoot, state.pose.x - goal_x);
        }
        require(reached, "closed loop did not converge");
        require(overshoot <= config.goal_tolerance, "closed loop overshot position tolerance");
        std::cout << "goal=" << goal_x << " initial_speed=" << initial_speed << " final_x=" << state.pose.x
                  << " final_speed=" << state.velocity.vx << " overshoot=" << overshoot
                  << " max_plan_ms=" << slowest_ms << '\n';
    }
    void obstacleRoute(bool vehicle_defaults = false)
    {
        auto map = freeMap();
        for (double x = 1.3; x < 1.7; x += 0.05)
            for (double y = -0.25; y < 0.3; y += 0.05) block(map, x, y);
        const auto field = fieldOf(map);
        DwaConfig config;
        if (!vehicle_defaults)
        {
            config.car_geometry.radius = 0.2;
            config.limits.max_vx = 1.0;
            config.limits.max_vy = 0.6;
            config.limits.max_wz = 1.0;
        }
        const std::vector<Pose2D> path = vehicle_defaults
            ? std::vector<Pose2D>{{0,0,0}, {0.7,1.5,0}, {2.6,1.5,0}, {3.5,0,0}}
            : std::vector<Pose2D>{{0,0,0}, {0.8,0.9,0}, {2.2,0.9,0}, {3.0,0,0}};
        RobotState state;
        bool reached = false;
        double min_clearance = INFINITY;
        double max_plan_ms = 0.0;
        for (int i = 0; i < 800; ++i)
        {
            double nearest = INFINITY;
            for (std::size_t j = config.path_start_index; j + 1 < path.size(); ++j)
            {
                const double dx = path[j + 1].x - path[j].x, dy = path[j + 1].y - path[j].y;
                const double ratio = std::max(0.0, std::min(1.0,
                    ((state.pose.x - path[j].x) * dx + (state.pose.y - path[j].y) * dy) / (dx * dx + dy * dy)));
                const double distance = std::hypot(state.pose.x - path[j].x - ratio * dx,
                                                   state.pose.y - path[j].y - ratio * dy);
                if (distance < nearest) { nearest = distance; config.path_start_index = j; }
            }
            const auto begin = std::chrono::steady_clock::now();
            const auto result = runDwa(state, path, field, config);
            max_plan_ms = std::max(max_plan_ms, std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count());
            if (result.status == DwaStatus::GoalReached) { reached = true; break; }
            require(result.status == DwaStatus::Success, "obstacle route has no feasible command");
            state.velocity = result.best_velocity;
            advance(state, config.control_dt);
            if (i % 100 == 0) std::cout << "route step=" << i << " pose=" << state.pose.x << ',' << state.pose.y
                << ',' << state.pose.yaw << " velocity=" << state.velocity.vx << ',' << state.velocity.vy
                << ',' << state.velocity.wz << " cost=" << result.best_cost << " index=" << config.path_start_index << '\n';
            double clearance;
            require(queryClearance(state.pose.x, state.pose.y, field, config.car_geometry.radius, clearance)
                        && clearance > config.safety_margin, "obstacle route collided");
            min_clearance = std::min(min_clearance, clearance);
        }
        std::cout << "obstacle defaults=" << vehicle_defaults << " final=" << state.pose.x << ',' << state.pose.y
                  << " min_clearance=" << min_clearance << " max_plan_ms=" << max_plan_ms << '\n';
        require(reached, "obstacle route did not reach goal");
    }
}
int main()
{
    try
    {
        distanceAndSweep();
        departureAndProgress();
        mandatorySafety();
        closedLoop(0.12, 0.0);
        closedLoop(3.0, 0.0);
        closedLoop(7.0, 3.0);
        closedLoop(7.0, 3.0, 2);
        closedLoop(-0.15, 0.0);
        obstacleRoute();
        obstacleRoute(true);
        std::cout << "DWA regression checks passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
