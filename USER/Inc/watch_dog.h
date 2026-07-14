#ifndef __WATCH_DOG_H__
#define __WATCH_DOG_H__

#include <stdint.h>

#define RC_WATCHDOG_TIMEOUT_TICKS (20U)

void remote_control_watchdog_feed(void);
void remote_control_watchdog_update(void);
uint8_t remote_control_is_timeout(void);
void remote_control_enter_safe_state(void);

#endif