/* Include the real dispatcher in this test translation unit so production does
 * not need to expose its private state. Do not also link control_dispatcher.c separately. */
#ifdef CONTROL_TEST_LEGACY_LAYOUT
#include "control.c"
#else
#include "control_dispatcher.c"
#endif
#include "control_probe.h"

void ControlTest_ReadSnapshot(ControlTestSnapshot *snapshot) {
  snapshot->velocity_loop_count = s_ctx.velocity_loop_tick_count;
  snapshot->position_loop_count = s_ctx.position_loop_tick_count;
  snapshot->vel_set = s_ctx.vel_set;
  snapshot->traj = s_ctx.traj;
  snapshot->velocity = s_vel_limiter;
  snapshot->torque = s_torque_limiter;
}
