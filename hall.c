#include "hall.h"

#define HALL_TIMEOUT_TICKS  ((HALL_SYSCLK_HZ / 1000UL) * HALL_TIMEOUT_MS)

volatile uint16_t  hallRaw        = 0U;
volatile uint16_t  hallState      = 0U;
volatile int16_t   hallDir        = 0;
volatile float32_t hallElecHz     = 0.0f;
volatile float32_t hallRpm        = 0.0f;
volatile uint32_t  hallEdgeCount  = 0U;
volatile uint32_t  hallErrorCount = 0U;

volatile uint16_t hallSigA = 0U;
volatile uint16_t hallSigB = 2U;
volatile uint16_t hallSigC = 4U;

#pragma DATA_SECTION(hallLogA, "ramgs1")
#pragma DATA_SECTION(hallLogB, "ramgs1")
#pragma DATA_SECTION(hallLogC, "ramgs1")
uint16_t hallLogA[HALL_LOG_LEN];
uint16_t hallLogB[HALL_LOG_LEN];
uint16_t hallLogC[HALL_LOG_LEN];

volatile uint16_t hallLogArm  = 1U;     // 0 idle, 1 armed, 2 running
volatile uint16_t hallLogDiv  = 10U;    // 1 ms/sample
volatile uint16_t hallLogDone = 0U;
static volatile uint16_t logIdx = 0U;

static volatile uint32_t edgeTicks[6];
static volatile uint16_t edgeIdx  = 0U;
static volatile uint16_t edgeFill = 0U;
static volatile uint16_t primed   = 0U;
static volatile uint32_t lastEdgeTime = 0U;

// Forward sequence 1-3-2-6-4-5
static const uint16_t nextFwd[8] = { 0U, 3U, 6U, 2U, 5U, 1U, 4U, 0U };
static const uint16_t nextRev[8] = { 0U, 5U, 3U, 1U, 6U, 4U, 2U, 0U };

static uint16_t readHall(void);
static void hallEdge(void);
static void initHallPin(uint32_t pin, uint32_t pinCfg);
__interrupt void hallAISR(void);
__interrupt void hallBISR(void);
__interrupt void hallCISR(void);
__interrupt void hallLogISR(void);

void Hall_init(void)
{
    initHallPin(HALL_A_GPIO, HALL_A_PINCFG);
    initHallPin(HALL_B_GPIO, HALL_B_PINCFG);
    initHallPin(HALL_C_GPIO, HALL_C_PINCFG);

    // Free-running timestamp (counts down)
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFFUL);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0U);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_RUNFREE);
    CPUTimer_disableInterrupt(CPUTIMER1_BASE);
    CPUTimer_startTimer(CPUTIMER1_BASE);
    lastEdgeTime = CPUTimer_getTimerCount(CPUTIMER1_BASE);

    hallRaw   = readHall();
    hallState = hallRaw;

    GPIO_setInterruptPin(HALL_A_GPIO, GPIO_INT_XINT1);
    GPIO_setInterruptPin(HALL_B_GPIO, GPIO_INT_XINT2);
    GPIO_setInterruptPin(HALL_C_GPIO, GPIO_INT_XINT3);
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_setInterruptType(GPIO_INT_XINT3, GPIO_INT_TYPE_BOTH_EDGES);

    Interrupt_register(INT_XINT1, &hallAISR);
    Interrupt_register(INT_XINT2, &hallBISR);
    Interrupt_register(INT_XINT3, &hallCISR);

    GPIO_enableInterrupt(GPIO_INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT2);
    GPIO_enableInterrupt(GPIO_INT_XINT3);

    Interrupt_enable(INT_XINT1);
    Interrupt_enable(INT_XINT2);
    Interrupt_enable(INT_XINT3);

    // Capture sampler timer
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_setPeriod(CPUTIMER2_BASE,
                       (HALL_SYSCLK_HZ / HALL_LOG_RATE_HZ) - 1UL);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0U);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);
    CPUTimer_setEmulationMode(CPUTIMER2_BASE,
                              CPUTIMER_EMULATIONMODE_RUNFREE);
    Interrupt_register(INT_TIMER2, &hallLogISR);
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
    Interrupt_enable(INT_TIMER2);
    CPUTimer_startTimer(CPUTIMER2_BASE);
}

// Speed calc, call from main loop
void Hall_update(void)
{
    uint32_t now;
    uint32_t sinceLast;
    uint32_t sum = 0U;
    uint16_t fill;
    uint16_t i;
    int16_t  dir;

    hallRaw = readHall();

    hallSigA = (hallRaw & 1U);
    hallSigB = ((hallRaw >> 1U) & 1U) + 2U;
    hallSigC = ((hallRaw >> 2U) & 1U) + 4U;

    // Snapshot shared edge data
    DINT;
    now       = CPUTimer_getTimerCount(CPUTIMER1_BASE);
    sinceLast = lastEdgeTime - now;
    fill      = edgeFill;
    dir       = hallDir;
    for(i = 0U; i < fill; i++)
    {
        sum += edgeTicks[i];
    }
    if(sinceLast > HALL_TIMEOUT_TICKS)
    {
        primed   = 0U;
        edgeFill = 0U;
        hallDir  = 0;
        fill     = 0U;
    }
    EINT;

    if((fill == 0U) || (sum == 0U))
    {
        hallElecHz = 0.0f;
        hallRpm    = 0.0f;
        return;
    }

    // 6 edges = 1 elec rev
    hallElecHz = ((float32_t)HALL_SYSCLK_HZ * (float32_t)fill) /
                 ((float32_t)sum * 6.0f);
    hallRpm    = (hallElecHz * 60.0f / (float32_t)MOTOR_POLE_PAIRS) *
                 (float32_t)dir;
}

// Returns CBA
static uint16_t readHall(void)
{
    return (uint16_t)( GPIO_readPin(HALL_A_GPIO)        |
                      (GPIO_readPin(HALL_B_GPIO) << 1U) |
                      (GPIO_readPin(HALL_C_GPIO) << 2U));
}

uint16_t Hall_read(void)
{
    return readHall();
}

static void hallEdge(void)
{
    uint32_t now      = CPUTimer_getTimerCount(CPUTIMER1_BASE);
    uint16_t newState = readHall();
    uint16_t oldState = hallState;
    int16_t  newDir;

    if(newState == oldState)
    {
        return;                         // glitch
    }

    if(hallLogArm == 1U)
    {
        logIdx      = 0U;
        hallLogDone = 0U;
        hallLogArm  = 2U;
    }

    if((newState == 0U) || (newState == 7U))
    {
        hallErrorCount++;
        primed   = 0U;
        edgeFill = 0U;
        return;
    }

    hallState = newState;
    hallEdgeCount++;

    if(newState == nextFwd[oldState])
    {
        newDir = 1;
    }
    else if(newState == nextRev[oldState])
    {
        newDir = -1;
    }
    else
    {
        // Skipped state
        hallErrorCount++;
        primed       = 0U;
        edgeFill     = 0U;
        lastEdgeTime = now;
        return;
    }

    if(newDir != hallDir)
    {
        hallDir  = newDir;
        primed   = 0U;
        edgeFill = 0U;
    }

    if(primed != 0U)
    {
        edgeTicks[edgeIdx] = lastEdgeTime - now;
        edgeIdx = (edgeIdx + 1U) % 6U;
        if(edgeFill < 6U)
        {
            edgeFill++;
        }
    }
    primed       = 1U;
    lastEdgeTime = now;
}

static void initHallPin(uint32_t pin, uint32_t pinCfg)
{
    GPIO_setPinConfig(pinCfg);
    GPIO_setDirectionMode(pin, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(pin, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(pin, GPIO_QUAL_6SAMPLE);
    GPIO_setQualificationPeriod(pin, 100U);     // ~5 us filter
}

__interrupt void hallAISR(void)
{
    hallEdge();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void hallBISR(void)
{
    hallEdge();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void hallCISR(void)
{
    hallEdge();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP12);
}

__interrupt void hallLogISR(void)
{
    static uint16_t divCnt = 0U;
    uint16_t s;

    if(hallLogArm != 2U)
    {
        divCnt = 0U;
        return;
    }

    divCnt++;
    if(divCnt < hallLogDiv)
    {
        return;
    }
    divCnt = 0U;

    s = readHall();
    hallLogA[logIdx] = (s & 1U);
    hallLogB[logIdx] = ((s >> 1U) & 1U) + 2U;
    hallLogC[logIdx] = ((s >> 2U) & 1U) + 4U;

    logIdx++;
    if(logIdx >= HALL_LOG_LEN)
    {
        hallLogArm  = 0U;
        hallLogDone = 1U;
    }
}
