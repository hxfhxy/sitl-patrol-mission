#pragma once

namespace sitl_adapters {

// 记录「Goal 已发出，但句柄可能还没回来」。cancel 不能只看句柄是否非空。
class GoalSlot {
public:
  enum class CancelAction {
    None,          // 当前没有在飞的 Goal。
    Armed,         // 句柄还没到，先记下，句柄一到就取消。
    CancelHandle   // 句柄已经在，调用方现在就 async_cancel。
  };

  void begin_send()
  {
    phase_ = Phase::WaitingHandle;
    cancel_requested_ = false;
  }

  // 句柄到达。返回 true 表示等待期间已经请求过取消，调用方要立刻 cancel 这个句柄。
  bool handle_arrived_should_cancel()
  {
    phase_ = Phase::Active;
    return cancel_requested_;
  }

  void finish()
  {
    phase_ = Phase::Idle;
    cancel_requested_ = false;
  }

  CancelAction request_cancel()
  {
    if (phase_ == Phase::Idle) {
      return CancelAction::None;
    }
    cancel_requested_ = true;
    if (phase_ == Phase::WaitingHandle) {
      return CancelAction::Armed;
    }
    return CancelAction::CancelHandle;
  }

private:
  enum class Phase {
    Idle,           // 没有在飞的 Goal。
    WaitingHandle,  // async_send_goal 已返回，句柄还没到。
    Active          // 句柄已接受。
  };

  Phase phase_{Phase::Idle};
  bool cancel_requested_{false};
};

}  // namespace sitl_adapters
