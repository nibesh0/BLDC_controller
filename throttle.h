#ifndef THROTTLE_H
#define THROTTLE_H

#include "driverlib.h"
#include "device.h"

#define THR_ADC_BASE        ADCA_BASE
#define THR_ADC_RESULT      ADCARESULT_BASE
#define THR_ADC_CH          ADC_CH_ADCIN14  //  "A14"
#define THR_VREF            3.3f        // thrVolts display only, not used for scaling

#define THR_RPM_MAX         1800.0f     // full throttle
#define THR_FILT_HZ         10.0f
#define THR_UPDATE_HZ       1000.0f     //  match Throttle_update() call rate

extern volatile uint16_t  thrEnable;    // 1 = throttle sets rpmRef
extern volatile uint16_t  thrRawMin;    // ADC counts at 0%
extern volatile uint16_t  thrRawMax;    // ADC counts at 100%

extern volatile uint16_t  thrRaw;
extern volatile float32_t thrVolts;
extern volatile float32_t thrPct;       // 0..1
extern volatile float32_t thrRpm;

void Throttle_init(void);
void Throttle_update(void);

#endif
