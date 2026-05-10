#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Mini Gimbal module for small gimbal control
constructor_args:
  - task_stack_depth: 1536
  - pid_pit_angle:
      k: 1.0
      p: 1.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_pit_omega:
      k: 1.0
      p: 5.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_scope_angle:
      k: 1.0
      p: 1.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_scope_omega:
      k: 1.0
      p: 0.5
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_pitch: '@&motor_small_pit'
  - motor_scope: '@&motor_scope'
  - referee: '@&ref'
  - scope_open_angle: 0.0
  - thread_priority: LibXR::Thread::Priority::MEDIUM
template_args: []
required_hardware: []
depends:
  - qdu-future/CMD
  - qdu-future/Motor
  - qdu-future/BMI088
  - qdu-future/Referee
=== END MANIFEST === */
// clang-format on

#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

enum class MiniGimbalEvent : uint8_t {
  SET_MODE_RELAX,
  SET_MODE_COMMON,
  SET_MODE_LOB,
  RESET_LOB_MODE,
  RESET_MINIGIMBAL,
  SET_SCOPE_OPEN,
  SET_SCOPE_CLOSE,
};

enum class PitMode : uint8_t {
  PITRELAX,
  COMMON,
  LOB,
};

enum class ScopeMode : uint8_t {
  SCOPERELAX,
  OPEN,
  CLOSE,
};

constexpr uint16_t UI_GIMBAL_LAYER = 1;

class MiniGimbal : public LibXR::Application {
 public:
  MiniGimbal(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_pit_angle,
      LibXR::PID<float>::Param pid_pit_omega,
      LibXR::PID<float>::Param pid_scope_angle,
      LibXR::PID<float>::Param pid_scope_omega, Motor* motor_small_pitch,
      Motor* motor_scope, float scope_open_angle, Referee* referee,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      : pid_pit_angle_(pid_pit_angle),
        pid_pit_omega_(pid_pit_omega),
        pid_scope_angle_(pid_scope_angle),
        pid_scope_omega_(pid_scope_omega),
        motor_small_pitch_(motor_small_pitch),
        motor_scope_(motor_scope),
        scope_open_angle_(scope_open_angle),
        referee_(referee) {
    UNUSED(app);

    thread_.Create(this, ThreadFunc, "MiniGimbalThread", task_stack_depth,
                   thread_priority);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, MiniGimbal* minigimbal, uint32_t event_id) {
          UNUSED(in_isr);
          switch (static_cast<MiniGimbalEvent>(event_id)) {
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
              if (  //(minigimbal->scope_mode_ == ScopeMode::CLOSE) &&
                  minigimbal->pit_mode_ == PitMode::COMMON) {
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

    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_RELAX), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_COMMON), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::SET_MODE_LOB), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::RESET_LOB_MODE), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::RESET_MINIGIMBAL), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::SET_SCOPE_OPEN), callback);
    minigimbal_event_.Register(
        static_cast<uint32_t>(MiniGimbalEvent::SET_SCOPE_CLOSE), callback);

    // void (*DrawUi)(MiniGimbal*) = [](MiniGimbal* minigimbal) {
    //   minigimbal->DrawUI();
    // };
    // ui_timer_handle_ =
    //     LibXR::Timer::CreateTask(DrawUi, this, 1000);  // 1Hz like
    // LibXR::Timer::Add(ui_timer_handle_);
    // LibXR::Timer::Start(ui_timer_handle_);
  }

  static void ThreadFunc(MiniGimbal* minigimbal) {
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "ahrs_euler");
    euler_suber.StartWaiting();

    minigimbal->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      if (euler_suber.Available()) {
        minigimbal->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
      }

      minigimbal->Update();
      minigimbal->Control();
      LibXR::Thread::Sleep(2);
    }
  }

  void Update() {
    const float LAST_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle;
    const float LAST_SCOPE_ANGLE = motor_scope_feedback_.abs_angle;

    motor_small_pitch_->Update();
    motor_scope_->Update();
    motor_small_pitch_feedback_ = motor_small_pitch_->GetFeedback();
    motor_scope_feedback_ = motor_scope_->GetFeedback();

    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    const float DELTA_PIT_ANGLE =
        motor_small_pitch_feedback_.abs_angle - LAST_PIT_ANGLE;
    pit_angle_ += DELTA_PIT_ANGLE / trig_gear_ratio_;

    const float DELTA_SCOPE_ANGLE =
        motor_scope_feedback_.abs_angle - LAST_SCOPE_ANGLE;
    scope_angle_ += DELTA_SCOPE_ANGLE / trig_gear_ratio_;

    if (init_flag_) {
      init_pit_angle_ = pit_angle_;
      init_scope_angle_ = scope_angle_;
      init_flag_ = false;
    }

    pit_ = euler_.Pitch();
    if (pit_ > M_PI) {
      pit_ = pit_ - 2 * M_PI;
    }
  }

  void Control() {
    if (pit_mode_ == PitMode::PITRELAX &&
        scope_mode_ == ScopeMode::SCOPERELAX) {
      motor_small_pitch_->Relax();
      motor_scope_->Relax();
      return;
    }

    float pit_out = 0.0f;
    float scope_out = 0.0f;

    if (pit_mode_ != PitMode::PITRELAX) {
      float pit_error = target_pit_ - pit_angle_;
      float target_pit_speed = pid_pit_angle_.Calculate(pit_error, 0.0f, dt_);
      pit_out = pid_pit_omega_.Calculate(
          target_pit_speed,
          motor_small_pitch_feedback_.velocity / motor_max_speed_, dt_);
    }

    if (scope_mode_ != ScopeMode::SCOPERELAX) {
      float scope_error = target_scope_ - scope_angle_;
      float target_scope_speed =
          pid_scope_angle_.Calculate(scope_error, 0.0f, dt_);
      scope_out = pid_scope_omega_.Calculate(
          target_scope_speed, motor_scope_feedback_.velocity / motor_max_speed_,
          dt_);
    }

    if (pit_mode_ == PitMode::PITRELAX) {
      pit_out = 0.0f;
    }

    if (scope_mode_ == ScopeMode::SCOPERELAX) {
      scope_out = 0.0f;
    }

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             float output) {
      auto motor_cmd = Motor::MotorCmd(
          {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = output});
      motor->Control(motor_cmd);
    };

    motor_control(motor_small_pitch_, motor_small_pitch_feedback_, pit_out);
    motor_control(motor_scope_, motor_scope_feedback_, scope_out);
  }

  void OnMonitor() override {}

  LibXR::Event& GetEvent() { return minigimbal_event_; }

  void SetPitMode(PitMode mode) {
    pid_pit_angle_.Reset();
    pid_pit_omega_.Reset();
    if (mode == PitMode::LOB) {
      // pit_ = pit_ + 0.0349f;
      pit_ = std::clamp(pit_, -0.78f, 0.0f);
      target_pit_ = init_pit_angle_ + pit_;
    } else {
      target_pit_ = init_pit_angle_;
    }
    pit_mode_ = mode;
  }

  void SetScopeMode(ScopeMode mode) {
    if (mode == scope_mode_) {
      return;
    }
    pid_scope_angle_.Reset();
    pid_scope_omega_.Reset();
    if (mode == ScopeMode::CLOSE) {
      target_scope_ = init_scope_angle_;
    } else {
      target_scope_ = init_scope_angle_ - scope_open_angle_;
    }
    scope_mode_ = mode;
  }

  void ResetLob() {
    // pit_ = pit_ + 0.0348f;
    pit_ = std::clamp(pit_, -0.78f, 0.0f);
    target_pit_ = init_pit_angle_ + pit_;
  }

  void ResetGimbal() {
    first_enter_time_ = LibXR::Timebase::GetMilliseconds();
    while ((LibXR::Timebase::GetMilliseconds() - first_enter_time_) < 800) {
      auto pit_cmd = Motor::MotorCmd(
          {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.1});
      auto scope_cmd = Motor::MotorCmd(
          {.mode = Motor::ControlMode::MODE_CURRENT, .velocity = 0.15});
      motor_small_pitch_->Control(pit_cmd);
      motor_scope_->Control(scope_cmd);
      LibXR::Thread::Sleep(2);
    }
    while ((LibXR::Timebase::GetMilliseconds() - first_enter_time_) < 1500) {
      motor_small_pitch_->Relax();
      motor_scope_->Relax();
      LibXR::Thread::Sleep(2);
    }

    const float LAST_PIT_ANGLE = motor_small_pitch_feedback_.abs_angle;
    const float DELTA_PIT_ANGLE =
        motor_small_pitch_feedback_.abs_angle - LAST_PIT_ANGLE;
    pit_angle_ += DELTA_PIT_ANGLE / trig_gear_ratio_;

    const float LAST_SCOPE_ANGLE = motor_scope_feedback_.abs_angle;
    const float DELTA_SCOPE_ANGLE =
        motor_scope_feedback_.abs_angle - LAST_SCOPE_ANGLE;
    scope_angle_ += DELTA_SCOPE_ANGLE / trig_gear_ratio_;

    init_pit_angle_ = pit_angle_;
    init_scope_angle_ = scope_angle_;
  }
  void DrawUI() {
    if (referee_ == nullptr) return;

    uint16_t robot_id = referee_->GetRobotID();
    uint16_t client_id = referee_->GetClientID(robot_id);

    // 首次绘制使用ADD，后续使用MODIFY
    Referee::UIFigureOp ADD_OP = Referee::UIFigureOp::UI_OP_MODIFY;
    if (this->ui_tick_ % 4 == 0) {
      ADD_OP = Referee::UIFigureOp::UI_OP_ADD;
    }

    // 云台俯仰线终点计算
    uint16_t pit_x =
        318 + static_cast<uint16_t>(
                  100 * cosf(-this->pit_angle_ - this->euler_.Pitch()));
    uint16_t pit_y =
        643 + static_cast<uint16_t>(
                  100 * sinf(-this->pit_angle_ - this->euler_.Pitch()));

    // 云台镜头线终点计算
    uint16_t scope_x =
        318 + static_cast<uint16_t>(
                  200 * cosf(-this->scope_angle_ - this->euler_.Pitch()));
    uint16_t scope_y =
        643 + static_cast<uint16_t>(
                  200 * sinf(-this->scope_angle_ - this->euler_.Pitch()));

    // 根据模式设置颜色
    auto pit_color = (pit_mode_ == PitMode::PITRELAX)
                         ? Referee::UIColor::UI_COLOR_ORANGE
                         : Referee::UIColor::UI_COLOR_CYAN;
    auto scope_color = (scope_mode_ == ScopeMode::SCOPERELAX)
                           ? Referee::UIColor::UI_COLOR_ORANGE
                           : Referee::UIColor::UI_COLOR_PINK;

    switch (ui_step_) {
      case 0: {
        // 绘制云台俯仰线
        Referee::UIFigure line1_fig{};
        referee_->FillLine(line1_fig, "MP", ADD_OP, UI_GIMBAL_LAYER, pit_color,
                           3, 318, 643, pit_x, pit_y);
        referee_->SendUIFigure(robot_id, client_id, line1_fig);
        break;
      }
      case 1: {
        // 绘制云台镜头线
        Referee::UIFigure line2_fig{};
        referee_->FillLine(line2_fig, "MS", ADD_OP, UI_GIMBAL_LAYER,
                           scope_color, 3, 318, 643, scope_x, scope_y);
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

  LibXR::Thread thread_;

  // UI members like HeroLauncher
  uint8_t ui_step_ = 0;
  uint8_t ui_tick_ = 0;
  LibXR::Timer::TimerHandle ui_timer_handle_;
};
