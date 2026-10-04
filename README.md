# MiniGimbal

小云台模块：控制一个小 pitch 电机和一个倍镜电机，用于吊射视角调整 / Mini gimbal Module controlling a small pitch motor and a scope motor for adjusting the lob-shot view

## 1. 模块作用 / Purpose

构造时，MiniGimbal 创建线程 `MiniGimbalThread`（栈深 `param.task_stack_depth`，优先级 `param.thread_priority`），每轮循环后休眠 2 ms。

角度：两个电机的角度都由 `abs_angle` 的增量除以固定减速比 36 累加得到，第一次刷新时的角度记为初始角（零点）。

控制：两个电机各为角度环与速度环串联，速度反馈为电机转速除以 14976 rpm 归一化后的值，以 `MODE_CURRENT` 下发。pitch 与 scope 都处于放松状态时，两个电机 `Relax()`；只有一路放松时，该路输出为 0。

事件：`GetEvent()` 返回的 `LibXR::Event` 注册了全局枚举 `MiniGimbalEvent` 的各个值。

| 事件 | 行为 |
| --- | --- |
| `SET_MODE_RELAX` | pitch 与 scope 都放松 |
| `SET_MODE_COMMON` | pitch 回到初始角 |
| `SET_MODE_LOB` | 吊射：pitch 目标为初始角加 clamp(云台 IMU pitch + 0.13, -0.78, 0) |
| `RESET_LOB_MODE` | 按当前 IMU pitch 重新计算吊射目标 |
| `RESET_MINIGIMBAL` | 在 `COMMON` 下有效：两个电机以固定电流驱动 0.8 s，放松到 1.5 s，再以当前角度作为新的初始角 |
| `SET_SCOPE_OPEN` | scope 转到初始角加 `scope_open_angle` |
| `SET_SCOPE_CLOSE` | scope 回到初始角 |

订阅 `param.euler_topic_name`（默认 `ahrs_euler`，`LibXR::EulerAngle<float>`），其 pitch 用于吊射目标。`DrawUI()` 每次调用在图层 1 上绘制 pitch 线或镜头线之一，两步循环。

Upon construction, MiniGimbal creates the thread `MiniGimbalThread` (stack depth `param.task_stack_depth`, priority `param.thread_priority`), which sleeps for 2 ms after each iteration.

Angles: the angle of each motor is accumulated from the increments of `abs_angle` divided by the fixed reduction ratio 36, and the angle at the first refresh is recorded as the initial angle (zero point).

Control: each motor runs an angle loop in series with a speed loop, with the speed feedback being the motor speed normalized by 14976 rpm, and the output is sent in `MODE_CURRENT`. When both pitch and scope are relaxed, both motors are `Relax()`ed; when only one is relaxed, that motor outputs 0.

Events: `GetEvent()` returns a `LibXR::Event` on which every value of the global enum `MiniGimbalEvent` is registered.

| Event | Behavior |
| --- | --- |
| `SET_MODE_RELAX` | Both pitch and scope are relaxed |
| `SET_MODE_COMMON` | Pitch returns to the initial angle |
| `SET_MODE_LOB` | Lob shot: the pitch target is the initial angle plus clamp(gimbal IMU pitch + 0.13, -0.78, 0) |
| `RESET_LOB_MODE` | Recompute the lob target from the current IMU pitch |
| `RESET_MINIGIMBAL` | Effective in `COMMON`: both motors are driven with a fixed current for 0.8 s and relaxed until 1.5 s, and the current angles become the new initial angles |
| `SET_SCOPE_OPEN` | Scope turns to the initial angle plus `scope_open_angle` |
| `SET_SCOPE_CLOSE` | Scope returns to the initial angle |

The Module subscribes to `param.euler_topic_name` (default `ahrs_euler`, `LibXR::EulerAngle<float>`), whose pitch is used for the lob target. Each call of `DrawUI()` draws one of the pitch line and the scope line on layer 1, in a cycle of two steps.

## 2. 构造接口 / Constructor

```cpp
MiniGimbal(Motor& motor_small_pitch,
           Motor& motor_scope,
           Referee& referee,
           const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `motor_small_pitch`：`Motor`，小 pitch 电机，例如 `RMMotor` 实例。
- `motor_scope`：`Motor`，倍镜电机。
- `referee`：`Referee` 实例，供 `DrawUI()` 使用。

配置参数（`Param`；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：线程栈深，默认 1536。
- `pid_pit_angle`：pitch 角度环，默认 `{.k = 1.0f, .p = 1.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `pid_pit_omega`：pitch 速度环，默认 `{.k = 1.0f, .p = 5.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `pid_scope_angle`：scope 角度环，默认 `{.k = 1.0f, .p = 1.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `pid_scope_omega`：scope 速度环，默认 `{.k = 1.0f, .p = 0.5f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `scope_open_angle`：倍镜打开时相对初始角的角度，单位 rad，默认 0。
- `thread_priority`：线程优先级，默认 `LibXR::Thread::Priority::MEDIUM`。
- `euler_topic_name`：订阅的姿态欧拉角 Topic 名称，默认 `"ahrs_euler"`，与姿态解算实例发布的名称一致。

Dependencies:

- `motor_small_pitch`: a `Motor` for the small pitch motor, for example an `RMMotor` instance.
- `motor_scope`: a `Motor` for the scope motor.
- `referee`: the `Referee` instance, used by `DrawUI()`.

Configuration parameters (`Param`; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: thread stack depth, default 1536.
- `pid_pit_angle`: pitch angle loop, default `{.k = 1.0f, .p = 1.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `pid_pit_omega`: pitch speed loop, default `{.k = 1.0f, .p = 5.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `pid_scope_angle`: scope angle loop, default `{.k = 1.0f, .p = 1.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `pid_scope_omega`: scope speed loop, default `{.k = 1.0f, .p = 0.5f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `scope_open_angle`: angle of the opened scope relative to the initial angle in rad, default 0.
- `thread_priority`: thread priority, default `LibXR::Thread::Priority::MEDIUM`.
- `euler_topic_name`: name of the subscribed attitude Euler angle Topic, default `"ahrs_euler"`, matching the name published by the attitude instance.

## 3. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.euler_topic_name`（默认 `ahrs_euler`） | 订阅 | `LibXR::EulerAngle<float>` | 云台姿态，pitch 用于吊射目标 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.euler_topic_name` (default `ahrs_euler`) | Subscribe | `LibXR::EulerAngle<float>` | Gimbal attitude, the pitch is used for the lob target |

## 4. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/MiniGimbal` 写入的实例，依赖填写为其他模块实例的 id：`motor_small_pitch` 与 `motor_scope` 取自 `QDU-Robomaster/RMMotor` 实例，`referee` 取自 `QDU-Robomaster/Referee` 实例，它们须在本实例之前列出；PID 与 `scope_open_angle` 按机构标定：

An instance written by `xrobot instance add QDU-Robomaster/MiniGimbal`, with the dependencies set to the ids of other Module instances: `motor_small_pitch` and `motor_scope` come from `QDU-Robomaster/RMMotor` instances and `referee` from a `QDU-Robomaster/Referee` instance, all listed before this instance; the PIDs and `scope_open_angle` are calibrated on the mechanism:

```yaml
modules:
  - module: QDU-Robomaster/MiniGimbal
    id: MiniGimbal_0
    args:
      - motor_small_pitch: motor_small_pit
      - motor_scope: motor_scope
      - referee: ref
      - param:
          task_stack_depth: 1536
          pid_pit_angle:
            k: 1.0f
            p: 1.0f
            i: 1.0f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 0.0f
            cycle: false
          pid_pit_omega:
            k: 1.0f
            p: 8.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          pid_scope_angle:
            k: 1.0f
            p: 0.5f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          pid_scope_omega:
            k: 1.0f
            p: 1.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          scope_open_angle: 1.5f
          thread_priority: LibXR::Thread::Priority::MEDIUM
          euler_topic_name: "ahrs_euler"
```

## 5. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/Motor`：两个电机的接口。
- `QDU-Robomaster/Referee`：`DrawUI()` 使用的裁判系统接口。
- LibXR。

硬件：一个小 pitch 电机和一个倍镜电机，均通过 `Motor` 实例接入。

Dependencies:

- `QDU-Robomaster/Motor`: the interface of the two motors.
- `QDU-Robomaster/Referee`: the referee system interface used by `DrawUI()`.
- LibXR.

Hardware: a small pitch motor and a scope motor, both attached through `Motor` instances.
