#ifndef BLDC_H
#define BLDC_H

#include "driverlib.h"
#include "device.h"

#define BLDC_TBCLK_HZ       50000000UL
#define BLDC_PWM_HZ         20000UL
#define BLDC_TBPRD          ((uint16_t)(BLDC_TBCLK_HZ / (2UL * BLDC_PWM_HZ)))   // up-down count
#define BLDC_DEAD_NS        3000UL
#define BLDC_DEAD_COUNTS    ((uint16_t)(BLDC_DEAD_NS / 20UL))

#define BLDC_DUTY_MAX       0.50f
#define BLDC_PRECHARGE_MS   10UL        // bootstrap charge
#define BLDC_STALL_MS       500UL

#define BLDC_STOP           0U
#define BLDC_PRECHARGE      1U
#define BLDC_RUN            2U
#define BLDC_FAULT          3U

#define FAULT_NONE          0U
#define FAULT_HALL          1U          // hall 0 or 7
#define FAULT_STALL         2U
#define FAULT_OVERSPEED     3U

extern volatile uint16_t  motorRun;
extern volatile int16_t   motorDir;     // +1 / -1
extern volatile uint16_t  commOffset;   // 0..5
extern volatile float32_t dutyStart;
extern volatile float32_t dutyTarget;
extern volatile float32_t dutyRampPerS;
extern volatile float32_t rpmLimit;

extern volatile uint16_t  speedMode;    // 0 duty, 1 PI
extern volatile float32_t rpmRef;
extern volatile float32_t rpmMin;       // throttle ref floor
extern volatile float32_t rpmRampPerS;
extern volatile float32_t piKp;
extern volatile float32_t piKi;
extern volatile float32_t rpmRefNow;
extern volatile float32_t rpmErr;
extern volatile float32_t piInteg;
extern volatile float32_t piFb;         // |rpm| used by PI
extern volatile float32_t piProp;       // Kp * err
extern volatile float32_t piOutRaw;     // before clamp
extern volatile uint32_t  piCount;      // PI updates
extern volatile uint16_t  speedIdle;    // 1 = zero ref, coasting

extern volatile uint16_t  motorState;
extern volatile uint16_t  faultCode;
extern volatile float32_t dutyNow;
extern volatile uint16_t  commStep;
extern volatile uint32_t  commCount;
extern volatile uint32_t  isrTicks;

void Bldc_init(void);

#endif
