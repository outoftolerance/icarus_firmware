#ifndef BalloonConfiguration_h
#define BalloonConfiguration_h

#include <IcarusConfiguration.h>

#define STROBE_CHANNEL_0 PWM_CHANNEL_0
#define STROBE_CHANNEL_1 PWM_CHANNEL_1
#define STROBE_CHANNEL_2 PWM_CHANNEL_2
#define STROBE_CHANNEL_3 PWM_CHANNEL_3

#define STROBE_COUNT 4

#define STROBE_PWM_OFF 1000
#define STROBE_PWM_ON 2000

#define WATCHDOG_TIMEOUT 5000
#define TELEMETRY_CHECK_INTERVAL 100
#define EXECUTION_LED_INTERVAL 1000

#endif