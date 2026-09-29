# MiniGimbal

小云台模块：控制一个小 pitch 电机和一个倍镜（scope）电机，用于英雄等机器人的吊射视角调整。

## 工作方式

- 构造时创建线程 `MiniGimbalThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`），
  每轮循环后休眠 2 ms。
- 角度：两个电机的角度都由 `abs_angle` 的增量除以固定减速比 36 累加得到；第一次刷新时的角度记为
  初始角（零点）。
- 控制：各自为角度环 + 速度环（速度反馈为电机转速除以 14976 rpm 归一化），以 `MODE_CURRENT`
  下发。pitch 与 scope 都处于放松状态时两个电机 `Relax()`；只有一路放松时该路输出 0。
- 事件：`GetEvent()` 返回的 `LibXR::Event` 注册了全局枚举 `MiniGimbalEvent` 的各个值：

  | 事件 | 行为 |
  | --- | --- |
  | `SET_MODE_RELAX` | pitch 与 scope 都放松 |
  | `SET_MODE_COMMON` | pitch 回到初始角 |
  | `SET_MODE_LOB` | 吊射：pitch 目标 = 初始角 + clamp(云台 IMU pitch + 0.13, -0.78, 0) |
  | `RESET_LOB_MODE` | 按当前 IMU pitch 重新计算吊射目标 |
  | `RESET_MINIGIMBAL` | 仅在 `COMMON` 下有效：两个电机以固定电流驱动 0.8 s、放松到 1.5 s，再以当前角度作为新的初始角；该过程在触发事件的调用者上下文中阻塞约 1.5 s |
  | `SET_SCOPE_OPEN` | scope 转到初始角 + `scope_open_angle` |
  | `SET_SCOPE_CLOSE` | scope 回到初始角 |

- 订阅 `param.euler_topic_name`（默认 `ahrs_euler`，`LibXR::EulerAngle<float>`），其 pitch 用于吊射目标。
- 类中有绘制裁判系统 UI 的 `DrawUI()`，但当前构造函数没有启动对应的定时任务。

## 依赖

- `QDU-Robomaster/Motor`：两个电机的抽象接口。
- `QDU-Robomaster/Referee`：`DrawUI()` 使用的裁判系统接口。
- `QDU-Robomaster/CMD`：列在 manifest 中，当前代码不使用。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
MiniGimbal(Motor& motor_small_pitch,
           Motor& motor_scope,
           Referee& referee,
           const Param& param = {...});
```

依赖：

- `motor_small_pitch`：`Motor`，小 pitch 电机（如 `RMMotor` 实例）。
- `motor_scope`：`Motor`，倍镜电机。
- `referee`：`Referee` 实例。

配置（`Param`；PID 为 `LibXR::PID<float>::Param`，字段 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 1536。
- `pid_pit_angle`：pitch 角度环，默认 `k = 1, p = 1`，其余为 0。
- `pid_pit_omega`：pitch 速度环，默认 `k = 1, p = 5`，其余为 0。
- `pid_scope_angle`：scope 角度环，默认 `k = 1, p = 1`，其余为 0。
- `pid_scope_omega`：scope 速度环，默认 `k = 1, p = 0.5`，其余为 0。
- `scope_open_angle`：倍镜打开时相对初始角的角度 (rad)，默认 0。
- `thread_priority`：线程优先级，默认 `MEDIUM`。
- `euler_topic_name`：订阅的姿态欧拉角 Topic，默认 `"ahrs_euler"`，须与姿态解算实例发布的名字一致。

## 使用

```sh
xrobot module add QDU-Robomaster/MiniGimbal
xrobot setup
xrobot instance add QDU-Robomaster/MiniGimbal
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖填为已列出的实例 id：

```yaml
modules:
  - module: QDU-Robomaster/MiniGimbal
    id: minigimbal_0
    args:
      - motor_small_pitch: motor_small_pit
      - motor_scope: motor_scope
      - referee: referee
      - param:
          task_stack_depth: '1536'
          pid_pit_angle:
            k: 1.0f
            p: 1.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          pid_pit_omega:
            k: 1.0f
            p: 5.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          pid_scope_angle:
            k: 1.0f
            p: 1.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          pid_scope_omega:
            k: 1.0f
            p: 0.5f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          scope_open_angle: 0.0f
          thread_priority: LibXR::Thread::Priority::MEDIUM
          euler_topic_name: '"ahrs_euler"'
```

所有依赖都是其他模块实例的 id，须在本实例之前列出：`motor_small_pit`、`motor_scope` 为
`QDU-Robomaster/RMMotor` 实例，`referee` 为 `QDU-Robomaster/Referee` 实例。本例没有需要 BSP 用
`XR_REGISTER` 注册的对象。`scope_open_angle` 需按机构设置。

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/MiniGimbal`
（在 BSP 中）打印当前的构造函数。
