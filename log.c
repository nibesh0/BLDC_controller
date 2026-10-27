// UART data logger. Frame format (all little endian):
//   A5 5A | type | len lo | len hi | payload[len] | sum8(type..payload)
//   type 'D': ver, nCh, tickHz u32, logDiv u16, { chType, nameLen, name }...
//   type 'S': seq u16, tick u32, channel values in table order
// tools/pi_logger.py decodes this and writes CSV.

#include "log.h"
#include "bldc.h"
#include "hall.h"
#include "throttle.h"

#define LOG_LSPCLK_HZ       (HALL_SYSCLK_HZ / 4UL)  // device.c: LSPCLK = SYSCLK/4
#define LOG_TICK_HZ         BLDC_PWM_HZ             // Log_isr() call rate

#define LOG_SYNC0           0xA5U
#define LOG_SYNC1           0x5AU
#define LOG_FRAME_DESC      0x44U       // 'D'
#define LOG_FRAME_DATA      0x53U       // 'S'
#define LOG_VERSION         1U
#define LOG_MASK            (LOG_SLOTS - 1U)
#define LOG_HDR_WORDS       3U          // seq, tick lo, tick hi

// Channel types, must match pi_logger.py
#define T_U16               0U
#define T_I16               1U
#define T_U32               2U
#define T_I32               3U
#define T_F32               4U

typedef struct
{
    const char    *name;
    volatile void *addr;
    uint16_t       type;
} LogChan;

// Add channels here, one line each. Order = CSV column order.
static const LogChan chTable[] =
{
    { "rpmRef",     &rpmRef,     T_F32 },   // raw setpoint (throttle)
    { "rpmRefNow",  &rpmRefNow,  T_F32 },   // ramped setpoint used by PI
    { "piFb",       &piFb,       T_F32 },   // |rpm| used by PI
    { "hallRpm",    &hallRpm,    T_F32 },   // signed, latest
    { "rpmErr",     &rpmErr,     T_F32 },
    { "piProp",     &piProp,     T_F32 },   // Kp * err
    { "piInteg",    &piInteg,    T_F32 },
    { "piOutRaw",   &piOutRaw,   T_F32 },   // before clamp
    { "dutyNow",    &dutyNow,    T_F32 },   // applied duty
    { "piKp",       &piKp,       T_F32 },
    { "piKi",       &piKi,       T_F32 },
    { "piCount",    &piCount,    T_U32 },   // PI updates
    { "motorState", &motorState, T_U16 },
    { "faultCode",  &faultCode,  T_U16 },
    { "speedMode",  &speedMode,  T_U16 },
    { "speedIdle",  &speedIdle,  T_U16 },
    { "commStep",   &commStep,   T_U16 },
};
#define LOG_NUM_CH          (sizeof(chTable) / sizeof(chTable[0]))

volatile uint16_t logEnable  = 1U;
volatile uint16_t logDiv     = 40U;
volatile uint16_t logNumCh   = 0U;
volatile uint32_t logSent    = 0U;
volatile uint32_t logDropped = 0U;

// ISR -> main loop sample queue
#pragma DATA_SECTION(slotBuf, "ramgs0")
static uint16_t slotBuf[LOG_SLOTS][LOG_SLOT_WORDS];
static volatile uint16_t slotHead = 0U;     // written by ISR only
static volatile uint16_t slotTail = 0U;     // written by main only
static uint16_t slotWords = LOG_HDR_WORDS;

static volatile uint32_t logTick = 0U;
static uint16_t divCnt = 0U;
static uint16_t seq    = 0U;

// One byte per word (C28x char is 16 bit)
#pragma DATA_SECTION(txBuf, "ramgs0")
static uint16_t txBuf[LOG_TX_BYTES];
static uint16_t txLen = 0U;
static uint16_t txIdx = 0U;
static uint16_t txSum = 0U;

static uint32_t descTick = 0U;
static uint16_t descDiv  = 0U;              // 0 forces first descriptor
static uint16_t descLen  = 0U;

static uint16_t nameLen(const char *s);
static void txStart(uint16_t type, uint16_t len);
static void txPut(uint16_t b);
static void txPut16(uint16_t v);
static void txEnd(void);
static void buildDesc(uint16_t div);
static void buildData(const uint16_t *w);
static uint16_t loadFrame(void);

void Log_init(void)
{
    uint16_t n;
    uint16_t words = LOG_HDR_WORDS;
    uint16_t dlen  = 8U;
    uint16_t w;
    uint16_t d;

    // Clip channel list to buffer sizes
    for(n = 0U; n < LOG_NUM_CH; n++)
    {
        w = (chTable[n].type <= T_I16) ? 1U : 2U;
        d = 2U + nameLen(chTable[n].name);
        if(((words + w) > LOG_SLOT_WORDS) ||
           ((dlen + d + 6U) > LOG_TX_BYTES))
        {
            break;
        }
        words += w;
        dlen  += d;
    }
    logNumCh  = n;
    slotWords = words;
    descLen   = dlen;

    GPIO_setPinConfig(LOG_RX_PINCFG);
    GPIO_setDirectionMode(LOG_RX_GPIO, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(LOG_RX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(LOG_RX_GPIO, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(LOG_TX_PINCFG);
    GPIO_setDirectionMode(LOG_TX_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(LOG_TX_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(LOG_TX_GPIO, GPIO_QUAL_ASYNC);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SCI_performSoftwareReset(LOG_SCI_BASE);
    SCI_setConfig(LOG_SCI_BASE, LOG_LSPCLK_HZ, LOG_BAUD,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE |
                   SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(LOG_SCI_BASE);
    SCI_resetTxFIFO(LOG_SCI_BASE);
    SCI_enableFIFO(LOG_SCI_BASE);
    SCI_enableModule(LOG_SCI_BASE);
    SCI_performSoftwareReset(LOG_SCI_BASE);
}

// Snapshot channels into the queue. Keep this short, runs at 20 kHz.
void Log_isr(void)
{
    uint16_t *w;
    uint16_t head;
    uint16_t next;
    uint16_t i;
    uint16_t n;
    uint32_t v;
    uint32_t tick;

    tick    = logTick + 1UL;
    logTick = tick;

    if((logEnable == 0U) || (++divCnt < logDiv))
    {
        return;
    }
    divCnt = 0U;
    seq++;                              // gap in seq = dropped sample

    head = slotHead;
    next = (head + 1U) & LOG_MASK;
    if(next == slotTail)
    {
        logDropped++;
        return;
    }

    w    = slotBuf[head];
    w[0] = seq;
    w[1] = (uint16_t)(tick & 0xFFFFUL);
    w[2] = (uint16_t)(tick >> 16);
    n    = LOG_HDR_WORDS;
    for(i = 0U; i < logNumCh; i++)
    {
        if(chTable[i].type <= T_I16)
        {
            w[n++] = *(volatile uint16_t *)chTable[i].addr;
        }
        else
        {
            v      = *(volatile uint32_t *)chTable[i].addr;  // f32 as raw bits
            w[n++] = (uint16_t)(v & 0xFFFFUL);
            w[n++] = (uint16_t)(v >> 16);
        }
    }
    slotHead = next;
}

// Feed the TX FIFO without blocking
void Log_service(void)
{
    while(SCI_getTxFIFOStatus(LOG_SCI_BASE) != SCI_FIFO_TX16)
    {
        if(txIdx >= txLen)
        {
            if(loadFrame() == 0U)
            {
                return;
            }
        }
        SCI_writeCharNonBlocking(LOG_SCI_BASE, txBuf[txIdx]);
        txIdx++;
    }
}

// Next frame: descriptor once per second or on logDiv change, else a sample
static uint16_t loadFrame(void)
{
    uint16_t tail = slotTail;
    uint16_t div  = logDiv;
    uint32_t now  = logTick;

    txIdx = 0U;
    txLen = 0U;

    if((logEnable != 0U) &&
       ((descDiv != div) || ((now - descTick) >= LOG_TICK_HZ)))
    {
        descDiv  = div;
        descTick = now;
        buildDesc(div);
    }
    else if(tail != slotHead)
    {
        buildData(slotBuf[tail]);
        slotTail = (tail + 1U) & LOG_MASK;
        logSent++;
    }
    return txLen;
}

static void buildDesc(uint16_t div)
{
    uint16_t i;
    uint16_t k;
    uint16_t len;

    txStart(LOG_FRAME_DESC, descLen);
    txPut(LOG_VERSION);
    txPut(logNumCh);
    txPut16((uint16_t)(LOG_TICK_HZ & 0xFFFFUL));
    txPut16((uint16_t)(LOG_TICK_HZ >> 16));
    txPut16(div);
    for(i = 0U; i < logNumCh; i++)
    {
        len = nameLen(chTable[i].name);
        txPut(chTable[i].type);
        txPut(len);
        for(k = 0U; k < len; k++)
        {
            txPut((uint16_t)chTable[i].name[k]);
        }
    }
    txEnd();
}

static void buildData(const uint16_t *w)
{
    uint16_t i;

    txStart(LOG_FRAME_DATA, slotWords * 2U);
    for(i = 0U; i < slotWords; i++)
    {
        txPut16(w[i]);
    }
    txEnd();
}

static uint16_t nameLen(const char *s)
{
    uint16_t n = 0U;

    while((n < LOG_NAME_MAX) && ((s[n] & 0xFFU) != 0U))
    {
        n++;
    }
    return n;
}

static void txStart(uint16_t type, uint16_t len)
{
    txLen = 0U;
    txBuf[txLen++] = LOG_SYNC0;
    txBuf[txLen++] = LOG_SYNC1;
    txSum = 0U;
    txPut(type);
    txPut16(len);
}

static void txPut(uint16_t b)
{
    b &= 0xFFU;
    txBuf[txLen++] = b;
    txSum += b;
}

static void txPut16(uint16_t v)
{
    txPut(v & 0xFFU);
    txPut(v >> 8);
}

static void txEnd(void)
{
    txBuf[txLen++] = txSum & 0xFFU;
}
