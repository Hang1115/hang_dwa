# DWA 独立纯算法库交接文档

> 历史交接快照（2026-10-23），保留当时的设计与迁移背景。
> 文中的目录、接口、构建状态和已知限制不代表当前版本；当前用法以 [README](../README.md) 和源码为准。

更新日期：2026-10-23。本文根据当前磁盘源码和本轮讨论整理，可供新对话直接接手。

## 1. 目标与当前授权范围

从现有 ROS 1 Noetic catkin 工作区中提取 DWA 算法，建立一个能够独立构建、测试和调用的 C++ 库，后续放到 GitHub，供其他项目复用。

第一版定位：全向底盘，控制量为车体系的 `vx / vy / wz`，圆形车体，二维栅格距离场，路径跟踪式 DWA。

本轮只创建交接文档，没有建立新仓库、迁移源码、修改算法或上传 GitHub。新对话的实施范围以用户届时的指令为准。

协作偏好：

- 用中文讨论，用户通常自己写代码，AI 负责方案、审查和解释；用户明确要求搭建或修改时再执行相应工作。
- 注释简洁，数学公式使用 LaTeX。
- 不为了封装库而重写算法、堆叠防御性分支或扩展功能。
- 用户此前自行处理当前 ROS 包的编译、链接和 CMake；独立库需要普通 CMake，但不要顺带改造原工作区。

## 2. 仓库放置与迁移方式

当前源码根目录：

```text
/home/hang/ROS_projects/badminton_ws
```

建议在工作区外新建普通 C++ 项目和独立 Git 仓库，不需要新建 ROS 工作区。以下目标路径只是建议，尚未创建或最终选定：

```text
/home/hang/projects/dwa
```

迁移顺序：先复制纯算法文件到新目录，独立验证；原工作区保留。等独立库可用后，再单独处理 ROS 包对它的依赖，避免长期维护两份算法。

当前工作区有大量未提交修改和未跟踪文件。迁移应以磁盘上的最新文件为准，不能仅从 Git HEAD 导出，否则会漏掉新算法文件。不要清理、覆盖、暂存或提交用户的其他改动。

如果新目录不在当前会话可写范围内，使用合适的工作目录或权限机制；不要因此擅自把库建到其他位置。

## 3. 源文件清单与边界

以下路径均相对于当前源码根目录。

需要提取的文件：

| 路径 | 内容 |
| --- | --- |
| `src/badminton_dwa/include/badminton_dwa/dwa_types.hpp` | 基础类型、配置、评价函数指针、结果 |
| `src/badminton_dwa/include/badminton_dwa/dwa_core.hpp` | `runDwa` 公共接口 |
| `src/badminton_dwa/include/badminton_dwa/dwa_costs.hpp` | 评价上下文、总代价接口 |
| `src/badminton_dwa/include/badminton_dwa/dwa_map.hpp` | 坐标转换、距离场构建与查询接口 |
| `src/badminton_dwa/src/dwa_core.cpp` | 动态窗口、速度采样、轨迹预测、最优候选选择 |
| `src/badminton_dwa/src/dwa_costs.cpp` | 路径、进度、障碍代价和加权聚合 |
| `src/badminton_dwa/src/dwa_map.cpp` | 栅格坐标转换和 chamfer 距离场 |

保留在 ROS 适配层，不加入纯算法库：

- `dwa_node.hpp/.cpp`、`dwa_main.cpp`、`dwa_parameters.cpp`。
- `dwa_ros_conversions.hpp`。
- ROS 消息转换、坐标系检查/变换、时间戳、超时、订阅发布、参数读取、任务取消和停车逻辑。
- `check_navigation_integration.py` 等 ROS 集成验证。

`dwa_tracking.hpp/.cpp` 已由用户删除，不要恢复，也不要继续假设 `selectLocalPath` 存在。

建议的新仓库布局，名称可以在实施时确定：

```text
dwa/
├── include/badminton_dwa/
│   ├── dwa_types.hpp
│   ├── dwa_core.hpp
│   ├── dwa_costs.hpp
│   └── dwa_map.hpp
├── src/
│   ├── dwa_core.cpp
│   ├── dwa_costs.cpp
│   └── dwa_map.cpp
├── examples/
├── tests/
├── CMakeLists.txt
├── README.md
└── LICENSE
```

第一步可以保留 `badminton_dwa` 命名空间和 include 路径，减少迁移变化。仓库名称、许可证尚未确定，不要假定已选定。

## 4. 已确定的接口组织

### 4.1 runDwa

当前头文件与实现的入口一致：

```cpp
DwaResult runDwa(
    const RobotState& robot_state,
    const std::vector<Pose2D>& global_path,
    const DistanceField& field_map,
    const DwaConfig& config);
```

当前执行流程已经实现，不再是空函数：

```text
计算动态窗口 → 采样候选速度
                         ↓
构造一次 CostInput，引用全局路径、距离场和配置
                         ↓
遍历候选：预测轨迹 → 计算一次总代价 → 更新最优结果
```

无候选或没有候选优于初始的正无穷代价时，返回默认 `success = false` 的结果。

用户决定暂时直接使用完整全局路径。不要在本次封装中加入参考路径裁剪、跟踪进度状态或局部路径窗口。

### 4.2 配置与评价输入

- `DwaConfig` 持有 `MotionLimits`、`CarGeometry`、`std::vector<CostTerm>` 和时间、采样等参数。
- `CostTerm` 保存函数指针和权重；旧的 `CostWeights` 已删除。
- `CostInput` 是每次 `runDwa` 调用期间的共享上下文。
- 预测轨迹单独作为参数传入各评价函数，不放回 `CostInput`。

当前 `CostInput`：

```cpp
struct CostInput
{
    const std::vector<Pose2D>& reference_path;
    const DistanceField& distance_field;
    const CarGeometry& car_geometry;
    const std::vector<CostTerm>& cost_terms;
    const double d_inf;
};
```

其中 `reference_path` 当前引用传入的完整 `global_path`，`d_inf` 来自 `config.influence_distance`。

配置和输入数据由调用方持有，`CostInput` 不复制大对象，也不跨调用缓存。评价函数表可以跨控制周期保存，`runDwa` 只读取它。无需为生命周期另外引入规划器类。

## 5. 数据和数学约定

- `RobotState.pose` 使用统一规划坐标系；`velocity` 使用车体坐标系。地图原点、路径和轨迹位姿使用同一规划坐标系。
- 长度为米，时间为秒，角度为弧度，角速度为弧度/秒。
- 圆形车体半径保存在 `config.car_geometry.radius`，调用方应按实际车体和所需余量设置。
- `GridMap` 和 `DistanceField` 使用相同的 `MapGeometry`，行优先，`index = row * width + col`。
- `origin` 是格子 `(0,0)` 的左下角，包含地图平面旋转；世界坐标减去原点、旋转 `-origin.yaw` 后再除分辨率并向下取整。
- 格子范围是半开区间，上/右边界算越界。
- 未知格和占据值大于等于阈值的格子作为障碍。
- 距离场是格子中心到最近障碍格中心的 chamfer 距离，单位米。地图更新时构建一次，不放进候选循环。
- 无障碍地图的距离可以为正无穷，`queryDistance` 此时仍返回 `true`；障碍代价可为零。
- 所有代价越小越好，正无穷表示候选不可行。

三项代价保留现有形式：

| 代价 | 定义 |
| --- | --- |
| 路径 | 轨迹点到参考折线的平均平方距离，单位 m² |
| 进度 | `-(s_end - s_start)`，单位米，前进为负 |
| 障碍 | `clearance = field_distance - radius - resolution * sqrt(0.5)`；净距非正为正无穷，大于等于影响距离为零，其余为 `(1 - clearance / d_inf)^2` |

聚合对每个分项先检查有限性，再乘权重，因此障碍项即使权重为零，也仍会拒绝返回正无穷的碰撞候选。

## 6. 独立构建时必须收尾的接口事项

截至本文日期，代码尚未完整对齐，不能把当前文件直接复制后就宣称构建通过。以下是新库最小接通所需的工作，不是算法功能扩展：

1. `dwa_types.hpp` 在使用 `CostInput` 定义 `CostFunc` 前缺少 `struct CostInput;` 前向声明。
2. `CostFunc` 的轨迹参数目前仍按值传递，三个单项代价实现已经改为 `const std::vector<Pose2D>&`，两者需要统一。
3. `dwa_costs.hpp` 的 `calculateTotalCost` 仍声明按值接收轨迹，实现已使用常量引用，需要同步。
4. 三个单项代价函数在 `.cpp` 中有定义，头文件尚未公开声明。外部调用方组装 `cost_terms` 时需要能引用它们。
5. 默认 `DwaConfig` 的运动限制为零，`cost_terms` 为空。最小示例需要显式设置可用的运动限制、半径、评价函数与权重。空评价表会直接得到总代价零，因此示例应完整装配三个评价项。
6. 注释仍有已经不存在的 `isvalid`、`robot_radius` 等旧字段说明，迁移时按现有接口修正即可。

可以优先统一为：

```cpp
struct CostInput;
using CostFunc = double (*)(
    const std::vector<Pose2D>& trajectory,
    const CostInput& input);
```

保持当前结构即可，不要求为了修复依赖顺序另拆多个配置头文件。

## 7. 用户决定暂缓的事项

下列问题已经讨论，用户决定先完成独立库，不作为迁移前置条件。记录为已知限制即可，不要在新对话反复要求先修完：

- 全局路径裁剪、前向投影窗口；折返和自交路径仍可能投影跳段。
- 车体中心在地图内但车体部分越出地图的检测。
- chamfer 高估距离、查询点偏离格子中心等带来的净距非保守问题。
- 相邻轨迹点之间的连续碰撞检测。当前使用离散点检查，用户计划依靠较密轨迹点先运行。
- 进度代价归一化，以及路径累计弧长等预计算优化。
- 平滑、终点朝向、额外运动模型和其他评价项。

这些是尚未解决的限制，不应在 README 中描述为已经具有严格连续碰撞保证。第一版验收聚焦独立构建、接口可调用和现有算法行为。

## 8. 新对话建议执行顺序与验收

在用户明确要求实施建立独立库后：

1. 读取本文及磁盘上的最新七个算法文件，确认目标目录，以复制方式建立新项目，保留原工作区。
2. 修复第 6 节列出的最小接口问题，保持算法行为不变。
3. 使用普通 CMake 创建算法库和示例目标。现有工程为 C++14，可先沿用；不引入 ROS/catkin 依赖。
4. 编写最小示例：构造地图并构建距离场，构造直线参考路径和机器人状态，设置配置与三项评价规则，运行 `runDwa` 并输出结果。
5. 补充与新接口匹配的基础测试，至少验证直线前进、明显碰撞候选被拒绝、无可行候选时失败；原有旧接口测试不能原样宣称通过。
6. 验证公共头文件可以独立包含，外部程序只通过公共头文件和链接库即可调用，不依赖包含 `.cpp`。
7. 补 README，说明模型、配置装配、单位/坐标约定、构建调用方式和已知限制；后续补安装导出和独立消费者验证，使其他项目便于接入。
8. 按新对话指令建立本地 Git 仓库；GitHub 上传、提交和许可证选定按实际授权处理。此文不代表已经授权发布。

第一阶段示例验收命令形式：

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

需要实际注册测试并执行示例，不能仅凭 `ctest` 无测试时返回成功判断验收通过。示例最终名称和运行命令由新项目确定。

现有 `test_core.cpp`、`test_dynamic_window.cpp`、`test_obstacle_cost.cpp` 多处仍使用旧接口；`test_tracking.cpp` 针对已删除的 tracking 模块。按用例意图选择性迁移，不恢复已删除模块。

## 9. 历史文档和验证状态

- `doc/dwa算法/cost_functions.md` 可参考代价推导，但其中部分额外评价项不属于本次范围。
- `doc/dwa算法/data_flow.md`、根目录 `AGENTS.md` / `CLAUDE.md` 的重构状态存在过时内容，例如仍说 `runDwa` 是空函数、仍依赖 tracking、仍存在旧权重结构。协作规则继续遵守，代码状态以最新源码和本文核对结果为准。
- 本轮没有运行整包编译或独立库构建；尚无独立库验收结果。
- 前轮曾在 `/tmp` 做过障碍代价的局部探针，复现第 7 节中的边界限制；这不等于算法库已经通过完整测试，也不要依赖这些临时文件继续工作。

## 10. 可直接发给新对话的开场指令

```text
请先读取：
/home/hang/ROS_projects/badminton_ws/doc/dwa算法/standalone_library_handoff.md

我现在要基于已有 DWA 代码建立独立纯 C++ 算法库。先按交接文档核对当前源码、迁移边界和最小接口收尾事项，再根据我在新对话中的具体要求推进。保留原 ROS 工作区；全局路径裁剪、距离场近似和连续碰撞检查暂缓，不作为封装前置条件。用中文交流。
```
