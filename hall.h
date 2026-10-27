#ifndef HALL_H
#define HALL_H

#include "driverlib.h"
#include "device.h"

#define HALL_A_GPIO         66U
#define HALL_B_GPIO         131U
#define HALL_C_GPIO         130U
#define HALL_A_PINCFG       GPIO_66_GPIO66
#define HALL_B_PINCFG       GPIO_131_GPIO131
#define HALL_C_PINCFG       GPIO_130_GPIO130

#define MOTOR_POLE_PAIRS    8U

#define HALL_SYSCLK_HZ      100000000UL //  actual SYSCLK
#define HALL_TIMEOUT_MS     5000UL       // zero-speed timeout

#define HALL_LOG_LEN        1000U
#define HALL_LOG_RATE_HZ    10000UL

// Plot signals: A=0/1, B=2/3, C=4/5
extern volatile uint16_t hallSigA;
extern volatile uint16_t hallSigB;
extern volatile uint16_t hallSigC;
extern uint16_t hallLogA[HALL_LOG_LEN];
extern uint16_t hallLogB[HALL_LOG_LEN];
extern uint16_t hallLogC[HALL_LOG_LEN];
extern volatile uint16_t hallLogArm;    // 1 = arm
extern volatile uint16_t hallLogDiv;
extern volatile uint16_t hallLogDone;

extern volatile uint16_t  hallRaw;      // pins CBA
extern volatile uint16_t  hallState;    // last valid state
extern volatile int16_t   hallDir;      // +1 / -1 / 0
extern volatile float32_t hallElecHz;
extern volatile float32_t hallRpm;      // mechanical, signed
extern volatile uint32_t  hallEdgeCount;
extern volatile uint32_t  hallErrorCount;

void     Hall_init(void);
void     Hall_update(void);
uint16_t Hall_read(void);

#endif
