#ifndef TrackerConfiguration_h
#define TrackerConfiguration_h

#include <IcarusConfiguration.h>

#define WATCHDOG_TIMEOUT 5000
#define EXECUTION_LED_INTERVAL 1000

#define PAN_SERVO_CHANNEL PWM_CHANNEL_0
#define TILT_SERVO_CHANNEL PWM_CHANNEL_1

#define PAN_SERVO_PWM_MIN  150  // this is the 'minimum' pulse length count (out of 4096) Left
#define PAN_SERVO_PWM_MAX  600  // this is the 'maximum' pulse length count (out of 4096) Right
#define TILT_SERVO_PWM_MIN  260 // Vertical
#define TILT_SERVO_PWM_MAX  470 // Horizontal

#endif