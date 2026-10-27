#ifndef LOG_H
#define LOG_H

#include "driverlib.h"
#include "device.h"

// SCIA -> XDS100v2 virtual COM port ("XDS100 Class USB Serial Port") on LAUNCHXL-F28379D
#define LOG_SCI_BASE        SCIA_BASE
#define LOG_TX_GPIO         42U
#define LOG_RX_GPIO         43U
#define LOG_TX_PINCFG       GPIO_42_SCITXDA
#define LOG_RX_PINCFG       GPIO_43_SCIRXDA

// Exact with LSPCLK 25 MHz (BRR = 4). Must match --baud in pi_logger.py
#define LOG_BAUD            625000UL

#define LOG_SLOTS           32U         // sample queue depth, power of 2
#define LOG_SLOT_WORDS      64U         // max words per sample
#define LOG_TX_BYTES        512U        // max frame size
#define LOG_NAME_MAX        24U

extern volatile uint16_t logEnable;     // 1 = stream
extern volatile uint16_t logDiv;        // ISR ticks per sample (40 = 500 Hz)
extern volatile uint16_t logNumCh;      // channels actually sent
extern volatile uint32_t logSent;
extern volatile uint32_t logDropped;    // queue full

void Log_init(void);
void Log_isr(void);                     // call from bldcISR
void Log_service(void);                 // call from main loop, non-blocking

#endif
