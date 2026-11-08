#pragma once
#include <stdint.h>
constexpr uint8_t GPT_TIMER = 0, TIMER_MODE_PERIODIC = 0;
struct timer_callback_args_t {};
extern void (*mockTick)(timer_callback_args_t*);
struct FspTimer {
  static int8_t get_available_timer(uint8_t&) { return 0; }
  bool begin(uint8_t, uint8_t, int8_t, float hz, float, void (*callback)(timer_callback_args_t*)) {
    if (hz != 1000.0f) return false;
    mockTick = callback;
    return true;
  }
  bool setup_overflow_irq() { return true; }
  bool open() { return true; }
  bool start() { return true; }
};
