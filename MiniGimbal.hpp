#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 小云台模块：控制一个小 pitch 电机和一个倍镜电机，用于吊射视角调整 / Mini gimbal Module controlling a small pitch motor and a scope motor for adjusting the lob-shot view
depends:
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/Referee
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <cstdlib>
#include <cstring>

#include "Motor.hpp"
#include "Referee.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

/**
 * @brief 小云台事件，数值同时是 `GetEvent()` 上注册的事件 ID。
 *        Mini gimbal events; the values are also the event IDs registered on
 *        `GetEvent()`.
 */
enum class MiniGimbalEvent : uint8_t
{
  SET_MODE_RELAX,    ///< pitch 与 scope 都放松 Relax both pitch and scope
  SET_MODE_COMMON,   ///< pitch 回到初始角 Pitch returns to the initial angle
  SET_MODE_LOB,      ///< 吊射 Lob shot
  RESET_LOB_MODE,    ///< 按当前 IMU pitch 重新计算吊射目标
                     ///< Recompute the lob target from the current IMU pitch
  RESET_MINIGIMBAL,  ///< 重置初始角，仅在 COMMON 下有效
                     ///< Reset the initial angles, effective only in COMMON
  SET_SCOPE_OPEN,    ///< scope 转到打开位置 Scope turns to the open position
  SET_SCOPE_CLOSE,   ///< scope 回到初始角 Scope returns to the initial angle
};

/**
 * @brief pitch 电机模式。
 *        Pitch motor modes.
 */
enum class PitMode : uint8_t
{
  PITRELAX,  ///< 放松 Relax
  COMMON,    ///< 回到初始角 Return to the initial angle
  LOB,       ///< 吊射 Lob shot
};

/**
 * @brief scope 电机模式。
 *        Scope motor modes.
 */
enum class ScopeMode : uint8_t
{
  SCOPERELAX,  ///< 放松 Relax
  OPEN,        ///< 打开 Open
  CLOSE,       ///< 关闭 Close
};

/// 小云台 UI 使用的图层编号
/// Layer number used by the mini gimbal UI
constexpr uint16_t UI_MINI_GIMBAL_LAYER = 1;

/**
 * @brief 小云台模块：控制一个小 pitch 电机和一个倍镜（scope）电机，用于吊射视角调整。
 *        Mini gimbal Module controlling a small pitch motor and a scope motor for
 *        adjusting the lob-shot view.
 */
class MiniGimbal
{
 public:
  /**
   * @brief 小云台配置参数。
   *        Mini gimbal configuration parameters.
   */
  struct Param
  {
    uint32_t task_stack_depth;                 ///< 线程栈深
                                               ///< Thread stack depth
    LibXR::PID<float>::Param pid_pit_angle;    ///< pitch 角度环 PID
                                               ///< Pitch angle-loop PID
    LibXR::PID<float>::Param pid_pit_omega;    ///< pitch 速度环 PID
                                               ///< Pitch speed-loop PID
    LibXR::PID<float>::Param pid_scope_angle;  ///< scope 角度环 PID
                                               ///< Scope angle-loop PID
    LibXR::PID<float>::Param pid_scope_omega;  ///< scope 速度环 PID
                                               ///< Scope speed-loop PID
    float scope_open_angle;                    ///< 倍镜打开时相对初始角的角度 (rad)
                             ///< Angle of the opened scope relative to the initial
                             ///< angle (rad)
    LibXR::Thread::Priority thread_priority;  ///< 线程优先级
                                              ///< Thread priority
    const char* euler_topic_name;             ///< 订阅的姿态欧拉角 Topic 名称
                                   ///< Name of the subscribed attitude Euler angle Topic
  };

  /**
   * @brief 构造 MiniGimbal，创建控制线程并注册事件。
   *        Construct MiniGimbal, create the control thread and register the events.
   *
   * @param motor_small_pitch 小 pitch 电机。
   *                          Small pitch motor.
   * @param motor_scope 倍镜电机。
   *                    Scope motor.
   * @param referee Referee 实例，供 `DrawUI()` 使用。
   *                Referee instance used by `DrawUI()`.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  MiniGimbal(Motor& motor_small_pitch, Motor& motor_scope, Referee& referee,
             const Param& param = {.task_stack_depth = 1536,
                                   .pid_pit_angle = {.k = 1.0f,
                                                     .p = 1.0f,
                                                     .i = 0.0f,
                                                     .d = 0.0f,
                                                     .i_limit = 0.0f,
                                                     .out_limit = 0.0f,
                                                     .cycle = false},
                                   .pid_pit_omega = {.k = 1.0f,
                                                     .p = 5.0f,
                                                     .i = 0.0f,
                                                     .d = 0.0f,
                                                     .i_limit = 0.0f,
                                                     .out_limit = 0.0f,
                                                     .cycle = false},
                                   .pid_scope_angle = {.k = 1.0f,
                                                       .p = 1.0f,
                                                       .i = 0.0f,
                                                       .d = 0.0f,
                                                       .i_limit = 0.0f,
                                                       .out_limit = 0.0f,
                                                       .cycle = false},
                                   .pid_scope_omega = {.k = 1.0f,
                                                       .p = 0.5f,
                                                       .i = 0.0f,
                                                       .d = 0.0f,
                                                       .i_limit = 0.0f,
                                                       .out_limit = 0.0f,
                                                       .cycle = false},
                                   .scope_open_angle = 0.0f,
                                   .thread_priority = LibXR::Thread::Priority::MEDIUM,
                                   .euler_topic_name = "ahrs_euler"})
      : pid_pit_angle_(param.pid_pit_angle),
        pid_pit_omega_(param.pid_pit_omega),
        pid_scope_angle_(param.pid_scope_angle),
        pid_scope_omega_(param.pid_scope_omega),
        motor_small_pitch_(&motor_small_pitch),
        motor_scope_(&motor_scope),
        scope_open_angle_(param.scope_open_angle),
        referee_(&referee)
  {
    euler_topic_name_ = param.euler_topic_name;
    thread_.Create(this, ThreadFunc, "MiniGimbalThread", param.task_stack_depth,
                   param.thread_priority);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, MiniGimbal* minigimbal, uint32_t event_id)
        {
          UNUSED(in_isr);
          switch (static_cast<MiniGimbalEvent>(event_id))
          {
            case MiniGimbalEvent::SET_MODE_RELAX:
              minigimbal->SetPitMode(PitMode::PITRELAX);
              minigimbal->SetScopeMode(ScopeMode::SCOPERELAX);
              break;
            case MiniGimbalEvent::SET_MODE_COMMON:
              minigimbal->SetPitMode(PitMode::COMMON);
              break;
            case MiniGimbalEvent::SET_MODE_LOB:
              minigimbal->SetPitMode(PitMode::LOB);
              break;
            case MiniGimbalEvent::RESET_LOB_MODE:
              minigimbal->ResetLob();
              break;
            case MiniGimbalEvent::RESET_MINIGIMBAL:
              if (minigimbal->pit_mode_ == PitMode::COMMON)
              {
                minigimbal->ResetGimbal();
              }
              break;
            case MiniGimbalEvent::SET_SCOPE_OPEN:
              minigimbal->SetScopeMode(ScopeMode::OPEN);
              break;
            case MiniGimbalEvent::SET_SCOPE_CLOSE:
              minigimbal->SetScopeMode(ScopeMode::CLOSE);
              break;
            default:
              break;
          }
        },
        this);

    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_RELAX),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_COMMON),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_LOB),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::RESET_LOB_MODE),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::RESET_MINIGIMBAL),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::SET_SCOPE_OPEN),
                               callback);
    minigimbal_event_.Register(static_cast<uint32_t>(MiniGimbalEvent::SET_SCOPE_CLOSE),
                               callback);
  }

  /**
   * @brief 控制线程函数：订阅姿态欧拉角，每 2 ms 执行一轮更新与控制。
   *        Control thread function that subscribes to the attitude Euler angles and
   *        runs one update and control iteration every 2 ms.
   *
   * @param minigimbal MiniGimbal 实例指针。
   *                   Pointer to the MiniGimbal instance.
   */
  static void ThreadFunc(MiniGimbal* minigimbal)
  {
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        minigimbal->euler_topic_name_);
    euler_suber.StartWaiting();

    minigimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true)
    {
      if (euler_suber.Available())
      {
        minigimbal->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }

      minigimbal->Update();
      minigimbal->Control();
      LibXR::Thread::Sleep(2);
    }
  }

  /**
   * @brief 刷新电机反馈，累加 pitch 与 scope 角度，并读取姿态 pitch。
   *        Refresh the motor feedback, accumulate the pitch and scope angles, and
   *        read the attitude pitch.
   */
  void Update()
  {
    const float LAST_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle;
    const float LAST_SCOPE_ANGLE = motor_scope_feedback_.abs_angle;

    motor_small_pitch_->Update();
    motor_scope_->Update();
    motor_small_pitch_feedback_ = motor_small_pitch_->GetFeedback();
    motor_scope_feedback_ = motor_scope_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    const float DELTA_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle - LAST_PIT_ANGLE;
    pit_angle_ += DELTA_PIT_ANGLE / trig_gear_ratio_;

    const float DELTA_SCOPE_ANGLE = motor_scope_feedback_.abs_angle - LAST_SCOPE_ANGLE;
    scope_angle_ += DELTA_SCOPE_ANGLE / trig_gear_ratio_;

    if (init_flag_)
    {
      init_pit_angle_ = pit_angle_;
      init_scope_angle_ = scope_angle_;
      init_flag_ = false;
    }

    pit_ = euler_.Pitch();
    if (pit_ > M_PI)
    {
      pit_ = pit_ - 2 * M_PI;
    }
  }

  /**
   * @brief 计算 pitch 与 scope 的角度环、速度环输出并以 `MODE_CURRENT` 下发；
   *        两路都放松时 `Relax()`。
   *        Compute the angle-loop and speed-loop outputs of pitch and scope and send
   *        them in `MODE_CURRENT`; `Relax()` when both are relaxed.
   */
  void Control()
  {
    if (pit_mode_ == PitMode::PITRELAX && scope_mode_ == ScopeMode::SCOPERELAX)
    {
      motor_small_pitch_->Relax();
      motor_scope_->Relax();
      return;
    }

    float pit_out = 0.0f;
    float scope_out = 0.0f;

    if (pit_mode_ != PitMode::PITRELAX)
    {
      float pit_error = target_pit_ - pit_angle_;
      float target_pit_speed = pid_pit_angle_.Calculate(pit_error, 0.0f, dt_);
      pit_out = pid_pit_omega_.Calculate(
          target_pit_speed, motor_small_pitch_feedback_.velocity / motor_max_speed_, dt_);
    }

    if (scope_mode_ != ScopeMode::SCOPERELAX)
    {
      float scope_error = target_scope_ - scope_angle_;
      float target_scope_speed = pid_scope_angle_.Calculate(scope_error, 0.0f, dt_);
      scope_out = pid_scope_omega_.Calculate(
          target_scope_speed, motor_scope_feedback_.velocity / motor_max_speed_, dt_);
    }

    if (pit_mode_ == PitMode::PITRELAX)
    {
      pit_out = 0.0f;
    }

    if (scope_mode_ == ScopeMode::SCOPERELAX)
    {
      scope_out = 0.0f;
    }

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb, float output)
    {
      auto motor_cmd =
          Motor::MotorCmd({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = output});
      motor->Control(motor_cmd);
    };

    motor_control(motor_small_pitch_, motor_small_pitch_feedback_, pit_out);
    motor_control(motor_scope_, motor_scope_feedback_, scope_out);
  }

  /**
   * @brief 获取小云台事件对象，`MiniGimbalEvent` 的各个值注册在其上。
   *        Get the mini gimbal event object on which every value of
   *        `MiniGimbalEvent` is registered.
   *
   * @return 事件对象的引用。
   *         Reference to the event object.
   */
  LibXR::Event& GetEvent() { return minigimbal_event_; }

  /**
   * @brief 设置 pitch 模式并复位 pitch 的 PID；`LOB` 时按 IMU pitch 计算吊射目标，
   *        其他模式回到初始角。
   *        Set the pitch mode and reset the pitch PIDs; `LOB` computes the lob target
   *        from the IMU pitch, and the other modes return to the initial angle.
   *
   * @param mode 目标 pitch 模式。
   *             Target pitch mode.
   */
  void SetPitMode(PitMode mode)
  {
    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    if (mode == PitMode::LOB)
    {
      pit_ = pit_ + 0.13f;
      pit_ = std::clamp(pit_, -0.78f, 0.0f);
      target_pit_ = init_pit_angle_ + pit_;
    }
    else
    {
      target_pit_ = init_pit_angle_;
    }
    pit_mode_ = mode;
  }

  /**
   * @brief 设置 scope 模式并复位 scope 的 PID；`CLOSE` 回到初始角，其他模式转到
   *        初始角加 `scope_open_angle`；模式不变时直接返回。
   *        Set the scope mode and reset the scope PIDs; `CLOSE` returns to the initial
   *        angle and the other modes turn to the initial angle plus
   *        `scope_open_angle`; returns directly when the mode is unchanged.
   *
   * @param mode 目标 scope 模式。
   *             Target scope mode.
   */
  void SetScopeMode(ScopeMode mode)
  {
    if (mode == scope_mode_)
    {
      return;
    }
    pid_scope_angle_.Reset();
    pid_scope_omega_.Reset();
    if (mode == ScopeMode::CLOSE)
    {
      target_scope_ = init_scope_angle_;
    }
    else
    {
      target_scope_ = init_scope_angle_ + scope_open_angle_;
    }
    scope_mode_ = mode;
  }

  /**
   * @brief 按当前 IMU pitch 重新计算吊射目标。
   *        Recompute the lob target from the current IMU pitch.
   */
  void ResetLob()
  {
    pit_ = pit_ + 0.13f;
    pit_ = std::clamp(pit_, -0.78f, 0.0f);
    target_pit_ = init_pit_angle_ + pit_;
  }

  /**
   * @brief 重置初始角：两个电机以固定电流驱动 0.8 s，放松到 1.5 s，再以当前角度作为
   *        新的初始角；阻塞调用者约 1.5 s。
   *        Reset the initial angles: drive both motors with a fixed current for 0.8 s,
   *        relax them until 1.5 s, and take the current angles as the new initial
   *        angles; blocks the caller for about 1.5 s.
   */
  void ResetGimbal()
  {
    first_enter_time_ = LibXR::Timebase::GetMilliseconds();
    while ((LibXR::Timebase::GetMilliseconds() - first_enter_time_) < 800)
    {
      auto pit_cmd =
          Motor::MotorCmd({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.1});
      auto scope_cmd =
          Motor::MotorCmd({.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.15});
      motor_small_pitch_->Control(pit_cmd);
      motor_scope_->Control(scope_cmd);
      LibXR::Thread::Sleep(2);
    }
    while ((LibXR::Timebase::GetMilliseconds() - first_enter_time_) < 1500)
    {
      motor_small_pitch_->Relax();
      motor_scope_->Relax();
      LibXR::Thread::Sleep(2);
    }

    const float LAST_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle;
    const float DELTA_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle - LAST_PIT_ANGLE;
    pit_angle_ += DELTA_PIT_ANGLE / trig_gear_ratio_;

    const float LAST_SCOPE_ANGLE = motor_scope_feedback_.abs_angle;
    const float DELTA_SCOPE_ANGLE = motor_scope_feedback_.abs_angle - LAST_SCOPE_ANGLE;
    scope_angle_ += DELTA_SCOPE_ANGLE / trig_gear_ratio_;

    init_pit_angle_ = pit_angle_;
    init_scope_angle_ = scope_angle_;
  }
  /**
   * @brief 在图层 1 上绘制一步裁判系统 UI（pitch 线与镜头线交替）。
   *        Draw one step of the referee system UI on layer 1, alternating between
   *        the pitch line and the scope line.
   */
  void DrawUI()
  {
    uint16_t robot_id = referee_->GetRobotID();
    uint16_t client_id = referee_->GetClientID(robot_id);

    // 首次绘制使用ADD，后续使用MODIFY
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_tick_ % 4 == 0)
    {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
    }

    // 云台俯仰线终点计算
    uint16_t pit_x =
        318 + static_cast<uint16_t>(100 * cosf(-this->pit_angle_ - this->euler_.Pitch()));
    uint16_t pit_y =
        643 + static_cast<uint16_t>(100 * sinf(-this->pit_angle_ - this->euler_.Pitch()));

    // 云台镜头线终点计算
    uint16_t scope_x = 318 + static_cast<uint16_t>(
                                 200 * cosf(-this->scope_angle_ - this->euler_.Pitch()));
    uint16_t scope_y = 643 + static_cast<uint16_t>(
                                 200 * sinf(-this->scope_angle_ - this->euler_.Pitch()));

    // 根据模式设置颜色
    auto pit_color = (pit_mode_ == PitMode::PITRELAX) ? Referee::UIColor::UI_COLOR_ORANGE
                                                      : Referee::UIColor::UI_COLOR_CYAN;
    auto scope_color = (scope_mode_ == ScopeMode::SCOPERELAX)
                           ? Referee::UIColor::UI_COLOR_ORANGE
                           : Referee::UIColor::UI_COLOR_PINK;

    switch (ui_step_)
    {
      case 0:
      {
        // 绘制云台俯仰线
        Referee::UIFigure line1_fig{};
        referee_->FillLine(line1_fig, "MP", ADD_OP, UI_MINI_GIMBAL_LAYER, pit_color, 3,
                           318, 643, pit_x, pit_y);
        referee_->SendUIFigure(robot_id, client_id, line1_fig);
        break;
      }
      case 1:
      {
        // 绘制云台镜头线
        Referee::UIFigure line2_fig{};
        referee_->FillLine(line2_fig, "MS", ADD_OP, UI_MINI_GIMBAL_LAYER, scope_color, 3,
                           318, 643, scope_x, scope_y);
        referee_->SendUIFigure(robot_id, client_id, line2_fig);
        break;
      }
      default:
        break;
    }

    this->ui_step_ = (this->ui_step_ + 1) % 2;
    this->ui_tick_++;
  }

 private:
  LibXR::PID<float> pid_pit_angle_;
  LibXR::PID<float> pid_pit_omega_;
  LibXR::PID<float> pid_scope_angle_;
  LibXR::PID<float> pid_scope_omega_;
  Motor* motor_small_pitch_;
  Motor* motor_scope_;

  Motor::Feedback motor_small_pitch_feedback_;
  Motor::Feedback motor_scope_feedback_;

  LibXR::EulerAngle<float> euler_;

  LibXR::Event minigimbal_event_;
  PitMode pit_mode_ = PitMode::PITRELAX;
  ScopeMode scope_mode_ = ScopeMode::SCOPERELAX;

  float scope_open_angle_ = 0.0f;
  float trig_gear_ratio_ = 36.0f;
  float motor_max_speed_ = 14976.0;

  Referee* referee_;

  float target_pit_ = 0.0f;
  float target_scope_ = 0.0f;
  float pit_angle_ = 0.0f;
  float scope_angle_ = 0.0f;
  float init_pit_angle_ = 0.0f;
  float init_scope_angle_ = 0.0f;
  float pit_ = 0.0f;

  bool init_flag_ = true;
  LibXR::MillisecondTimestamp first_enter_time_ = 0;

  float dt_ = 0.0f;
  LibXR::MicrosecondTimestamp last_wakeup_;
  LibXR::MicrosecondTimestamp last_online_time_;

  const char* euler_topic_name_ = nullptr;
  LibXR::Thread thread_;

  // UI 绘制步骤与计数
  uint8_t ui_step_ = 0;
  uint8_t ui_tick_ = 0;
};
