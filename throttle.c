#include "throttle.h"

volatile uint16_t  thrEnable = 1U;
volatile uint16_t  thrRawMin = 1145U;   // measured, released (0.7 V at pin)
volatile uint16_t  thrRawMax = 3435U;   // measured, full (2.2 V at pin)

volatile uint16_t  thrRaw   = 0U;
volatile float32_t thrVolts = 0.0f;
volatile float32_t thrPct   = 0.0f;
volatile float32_t thrRpm   = 0.0f;

static float32_t thrFilt = 0.0f;

// 1st-order low-pass coefficient
#define THR_ALPHA   ((6.2832f * THR_FILT_HZ) / \
                     (THR_UPDATE_HZ + (6.2832f * THR_FILT_HZ)))

void Throttle_init(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);

    // ADCCLK = 25 MHz
    ADC_setPrescaler(THR_ADC_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(THR_ADC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(THR_ADC_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(THR_ADC_BASE);
    DEVICE_DELAY_US(1000U);             // ADC power-up

    // SW trigger, 1 us sample window
    ADC_setupSOC(THR_ADC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY,
                 THR_ADC_CH, 99U);

    ADC_forceSOC(THR_ADC_BASE, ADC_SOC_NUMBER0);
    DEVICE_DELAY_US(10U);
    thrRaw  = ADC_readResult(THR_ADC_RESULT, ADC_SOC_NUMBER0);
    thrFilt = (float32_t)thrRaw;
}

// Call at THR_UPDATE_HZ
void Throttle_update(void)
{
    float32_t lo;
    float32_t span;
    float32_t pct;

    // Read previous conversion, then start next
    thrRaw = ADC_readResult(THR_ADC_RESULT, ADC_SOC_NUMBER0);
    ADC_forceSOC(THR_ADC_BASE, ADC_SOC_NUMBER0);

    thrFilt += THR_ALPHA * ((float32_t)thrRaw - thrFilt);
    thrVolts = thrFilt * (THR_VREF / 4095.0f);

    // Calibrated in ADC counts, independent of THR_VREF
    lo   = (float32_t)thrRawMin;
    span = (float32_t)thrRawMax - lo;
    if(span < 50.0f)
    {
        span = 50.0f;                   // avoid divide by ~0
    }
    pct = (thrFilt - lo) / span;
    if(pct < 0.0f)
    {
        pct = 0.0f;
    }
    if(pct > 1.0f)
    {
        pct = 1.0f;
    }

    thrPct = pct;
    thrRpm = pct * THR_RPM_MAX;         // pct already clamped to 0..1
}
