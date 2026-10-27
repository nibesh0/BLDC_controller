// Sensored BLDC 6-step: high-side PWM, low-side ON, third phase off.
// EPWMxA = low side, EPWMxB = high side
//   U: EPWM1 GPIO0 lo (J4-40), GPIO1 hi (J4-39), Hall A
//   V: EPWM2 GPIO2 lo (J4-38), GPIO3 hi (J4-37), Hall B
//   W: EPWM3 GPIO4 lo (J4-36), GPIO5 hi (J4-35), Hall C

#include "bldc.h"
#include "hall.h"
#include "throttle.h"
#include "log.h"

// AQCSFRC: CSFA bits 1:0, CSFB bits 3:2
#define LEG_OFF             0x5U        // lo off, hi off
#define LEG_LOW             0x6U        // lo on,  hi off
#define LEG_PWM             0x1U        // lo off, hi PWM

#define ISR_HZ              BLDC_PWM_HZ
#define PRECHARGE_TICKS     ((ISR_HZ / 1000UL) * BLDC_PRECHARGE_MS)
#define STALL_TICKS         ((ISR_HZ / 1000UL) * BLDC_STALL_MS)
#define CMPA_MIN            BLDC_DEAD_COUNTS

volatile uint16_t  motorRun     = 0U;
volatile int16_t   motorDir     = 1;
volatile uint16_t  commOffset   = 0U;
volatile float32_t dutyStart    = 0.10f;
volatile float32_t dutyTarget   = 0.15f;
volatile float32_t dutyRampPerS = 0.05f;
volatile float32_t rpmLimit     = 50000.0f;

volatile uint16_t  speedMode    = 1U;
volatile float32_t rpmRef       = 300.0f;
volatile float32_t rpmMin       = 200.0f;   // throttle ref floor
volatile float32_t rpmRampPerS  = 250.0f;
volatile float32_t piKp         = 0.00004f;  // duty/rpm
volatile float32_t piKi         = 0.00050f;  // duty/(rpm*s)
volatile float32_t rpmRefNow    = 0.0f;
volatile float32_t rpmErr       = 0.0f;
volatile float32_t piInteg      = 0.0f;
volatile float32_t piFb         = 0.0f;
volatile float32_t piProp       = 0.0f;
volatile float32_t piOutRaw     = 0.0f;
volatile uint32_t  piCount      = 0U;

volatile uint16_t  motorState   = BLDC_STOP;
volatile uint16_t  faultCode    = FAULT_NONE;
volatile float32_t dutyNow      = 0.0f;
volatile uint16_t  commStep     = 0xFFFFU;
volatile uint32_t  commCount    = 0U;
volatile uint32_t  isrTicks     = 0U;

static const uint32_t legBase[3] = { EPWM1_BASE, EPWM2_BASE, EPWM3_BASE };

// Hall state -> sector (0xFF invalid)
static const uint16_t hallSector[8] = { 0xFFU, 1U, 3U, 2U, 5U, 0U, 4U, 0xFFU };

// Step -> U, V, W modes
static const uint16_t stepTable[6][3] =
{
    { LEG_PWM, LEG_LOW, LEG_OFF },      // 0: U+ V-
    { LEG_PWM, LEG_OFF, LEG_LOW },      // 1: U+ W-
    { LEG_OFF, LEG_PWM, LEG_LOW },      // 2: V+ W-
    { LEG_LOW, LEG_PWM, LEG_OFF },      // 3: V+ U-
    { LEG_LOW, LEG_OFF, LEG_PWM },      // 4: W+ U-
    { LEG_OFF, LEG_LOW, LEG_PWM },      // 5: W+ V-
};

static uint32_t stateTicks = 0U;
static uint32_t stallTicks = 0U;
static uint16_t piActive   = 0U;
static uint16_t piDiv      = 0U;
static uint16_t thrDiv     = 0U;
volatile uint16_t speedIdle = 0U;

#define PI_DIV              20U         // 1 kHz
#define PI_TS               ((float32_t)PI_DIV / (float32_t)ISR_HZ)

static void initLegPWM(uint32_t base);
static void initLegGPIO(void);
static void setAllLegs(uint16_t mode);
static void setDuty(float32_t duty);
static void outputsOn(void);
static void outputsOff(void);
static void tripFault(uint16_t code);
static void dutyRamp(void);
static void speedPI(void);
__interrupt void bldcISR(void);

void Bldc_init(void)
{
    uint16_t i;

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_setEPWMClockDivider(SYSCTL_EPWMCLK_DIV_2);

    for(i = 0U; i < 3U; i++)
    {
        initLegPWM(legBase[i]);
    }
    initLegGPIO();

    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U);
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    EPWM_enableInterrupt(EPWM1_BASE);
    Interrupt_register(INT_EPWM1, &bldcISR);
    Interrupt_enable(INT_EPWM1);

    // Start all legs in sync
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

static void initLegPWM(uint32_t base)
{
    // Hold outputs low until outputsOn()
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);

    EPWM_setClockPrescaler(base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(base, BLDC_TBPRD);
    EPWM_setTimeBaseCounter(base, 0U);
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(base);
    EPWM_setEmulationMode(base, EPWM_EMULATION_STOP_AFTER_FULL_CYCLE);

    EPWM_setCounterCompareShadowLoadMode(base, EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_A,
                                BLDC_TBPRD + 1U);

    // High side (B): on above CMPA, pulse centered on period
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    // Low side (A): driven by software force only
    EPWM_setActionQualifierAction(base, EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    // Commutate at zero, where high side is off (dead time)
    EPWM_setActionQualifierContSWForceShadowMode(base,
                                                 EPWM_AQ_SW_SH_LOAD_ON_CNTR_ZERO);
    HWREGH(base + EPWM_O_AQCSFRC) = LEG_OFF;

    EPWM_setDeadBandDelayMode(base, EPWM_DB_RED, false);
    EPWM_setDeadBandDelayMode(base, EPWM_DB_FED, false);
}

static void initLegGPIO(void)
{
    uint16_t pin;

    for(pin = 0U; pin < 6U; pin++)
    {
        GPIO_setPadConfig(pin, GPIO_PIN_TYPE_STD);
    }
    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setPinConfig(GPIO_1_EPWM1B);
    GPIO_setPinConfig(GPIO_2_EPWM2A);
    GPIO_setPinConfig(GPIO_3_EPWM2B);
    GPIO_setPinConfig(GPIO_4_EPWM3A);
    GPIO_setPinConfig(GPIO_5_EPWM3B);
}

static void setAllLegs(uint16_t mode)
{
    HWREGH(EPWM1_BASE + EPWM_O_AQCSFRC) = mode;
    HWREGH(EPWM2_BASE + EPWM_O_AQCSFRC) = mode;
    HWREGH(EPWM3_BASE + EPWM_O_AQCSFRC) = mode;
}

static void setDuty(float32_t duty)
{
    uint16_t cmpa;
    uint16_t i;

    if(duty <= 0.0f)
    {
        cmpa = BLDC_TBPRD + 1U;         // never matches = off
    }
    else
    {
        cmpa = (uint16_t)((1.0f - duty) * (float32_t)BLDC_TBPRD);
        if(cmpa < CMPA_MIN)
        {
            cmpa = CMPA_MIN;            // keep dead time at zero
        }
    }

    for(i = 0U; i < 3U; i++)
    {
        EPWM_setCounterCompareValue(legBase[i], EPWM_COUNTER_COMPARE_A, cmpa);
    }
}

static void outputsOn(void)
{
    uint16_t i;

    for(i = 0U; i < 3U; i++)
    {
        EPWM_clearTripZoneFlag(legBase[i], EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
    }
}

static void outputsOff(void)
{
    uint16_t i;

    for(i = 0U; i < 3U; i++)
    {
        EPWM_forceTripZoneEvent(legBase[i], EPWM_TZ_FORCE_EVENT_OST);
    }
    setAllLegs(LEG_OFF);
    setDuty(0.0f);
    dutyNow  = 0.0f;
    commStep = 0xFFFFU;
}

static void tripFault(uint16_t code)
{
    outputsOff();
    faultCode  = code;
    motorRun   = 0U;
    motorState = BLDC_FAULT;
}

// 20 kHz control
__interrupt void bldcISR(void)
{
    uint16_t hall;
    uint16_t sector;
    uint16_t step;
    float32_t rpm;

    isrTicks++;
    hall = Hall_read();

    if(++thrDiv >= PI_DIV)
    {
        thrDiv = 0U;
        Throttle_update();
        if(thrEnable != 0U)
        {
            // Throttle ref never below rpmMin (C has no max())
            rpmRef = (thrRpm > rpmMin) ? thrRpm : rpmMin;
        }
    }

    switch(motorState)
    {
        case BLDC_STOP:
        case BLDC_FAULT:
            if(motorRun != 0U)
            {
                // All low sides on to charge bootstrap caps
                faultCode  = FAULT_NONE;
                setDuty(0.0f);
                setAllLegs(LEG_LOW);
                outputsOn();
                stateTicks = 0U;
                motorState = BLDC_PRECHARGE;
            }
            break;

        case BLDC_PRECHARGE:
            if(motorRun == 0U)
            {
                outputsOff();
                motorState = BLDC_STOP;
            }
            else if(++stateTicks >= PRECHARGE_TICKS)
            {
                setAllLegs(LEG_OFF);
                dutyNow    = dutyStart;
                stallTicks = 0U;
                commStep   = 0xFFFFU;
                piActive   = 0U;
                motorState = BLDC_RUN;
            }
            break;

        case BLDC_RUN:
            if(motorRun == 0U)
            {
                outputsOff();
                motorState = BLDC_STOP;
                break;
            }

            sector = hallSector[hall & 7U];
            if(sector == 0xFFU)
            {
                tripFault(FAULT_HALL);
                break;
            }

            rpm = hallRpm;
            if((rpm > rpmLimit) || (rpm < -rpmLimit))
            {
                tripFault(FAULT_OVERSPEED);
                break;
            }

            // Reverse = shift by 3 steps
            step = (sector + (commOffset % 6U) + ((motorDir < 0) ? 3U : 0U)) % 6U;
            if(step != commStep)
            {
                HWREGH(EPWM1_BASE + EPWM_O_AQCSFRC) = stepTable[step][0];
                HWREGH(EPWM2_BASE + EPWM_O_AQCSFRC) = stepTable[step][1];
                HWREGH(EPWM3_BASE + EPWM_O_AQCSFRC) = stepTable[step][2];
                commStep   = step;
                commCount++;
                stallTicks = 0U;
            }
            else if((speedIdle == 0U) && (++stallTicks >= STALL_TICKS))
            {
                tripFault(FAULT_STALL);
                break;
            }

            if(speedMode != 0U)
            {
                // Bumpless start
                if(piActive == 0U)
                {
                    piInteg   = dutyNow;
                    rpmRefNow = (rpm < 0.0f) ? -rpm : rpm;
                    piDiv     = 0U;
                    piActive  = 1U;
                }
                if(++piDiv >= PI_DIV)
                {
                    piDiv = 0U;
                    speedPI();
                }

                // Zero ref: coast
                if((rpmRef < 1.0f) && (rpmRefNow < 1.0f))
                {
                    dutyNow    = 0.0f;
                    piInteg    = 0.0f;
                    stallTicks = 0U;
                    speedIdle  = 1U;
                }
                else if(speedIdle != 0U)
                {
                    piInteg   = dutyStart;
                    dutyNow   = dutyStart;
                    speedIdle = 0U;
                }
            }
            else
            {
                piActive  = 0U;
                speedIdle = 0U;
                dutyRamp();
            }
            setDuty(dutyNow);
            break;

        default:
            tripFault(FAULT_NONE);
            break;
    }

    Log_isr();

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

// Open-loop duty ramp
static void dutyRamp(void)
{
    float32_t target = dutyTarget;
    float32_t step   = dutyRampPerS * (1.0f / (float32_t)ISR_HZ);

    if(target > BLDC_DUTY_MAX)
    {
        target = BLDC_DUTY_MAX;
    }
    if(target < 0.0f)
    {
        target = 0.0f;
    }

    if(dutyNow < target)
    {
        dutyNow += step;
        if(dutyNow > target)
        {
            dutyNow = target;
        }
    }
    else if(dutyNow > target)
    {
        dutyNow -= step;
        if(dutyNow < target)
        {
            dutyNow = target;
        }
    }
}

// Speed PI, 1 kHz
static void speedPI(void)
{
    float32_t ref  = rpmRef;
    float32_t step = rpmRampPerS * PI_TS;
    float32_t fb   = hallRpm;
    float32_t err;
    float32_t out;

    if(ref < 0.0f)
    {
        ref = 0.0f;
    }
    if(ref > rpmLimit)
    {
        ref = rpmLimit;
    }

    if(rpmRefNow < ref)
    {
        rpmRefNow += step;
        if(rpmRefNow > ref)
        {
            rpmRefNow = ref;
        }
    }
    else if(rpmRefNow > ref)
    {
        rpmRefNow -= step;
        if(rpmRefNow < ref)
        {
            rpmRefNow = ref;
        }
    }

    if(fb < 0.0f)
    {
        fb = -fb;
    }

    err    = rpmRefNow - fb;
    rpmErr = err;
    piFb   = fb;

    // Anti-windup clamp
    piInteg += piKi * err * PI_TS;
    if(piInteg > BLDC_DUTY_MAX)
    {
        piInteg = BLDC_DUTY_MAX;
    }
    if(piInteg < 0.0f)
    {
        piInteg = 0.0f;
    }

    piProp   = piKp * err;
    out      = piProp + piInteg;
    piOutRaw = out;
    if(out > BLDC_DUTY_MAX)
    {
        out = BLDC_DUTY_MAX;
    }
    if(out < 0.0f)
    {
        out = 0.0f;
    }

    dutyNow = out;
    piCount++;
}
