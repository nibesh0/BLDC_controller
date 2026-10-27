#include "driverlib.h"
#include "device.h"
#include "hall.h"
#include "bldc.h"
#include "throttle.h"
#include "log.h"

#define LED_TIMER_PERIOD    100000000UL

__interrupt void cpuTimer0ISR(void);

void main(void)
{
    Device_init();
    Device_initGPIO();

    ERTM;                               // allow realtime debug access

    Interrupt_initModule();
    Interrupt_initVectorTable();

    // LEDs are active low
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);
    GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1U);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);

    Hall_init();
    Throttle_init();
    Log_init();
    Bldc_init();

    // Heartbeat timer
    CPUTimer_setPeriod(CPUTIMER0_BASE, LED_TIMER_PERIOD);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0U);
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);

    EINT;
    ERTM;
    CPUTimer_startTimer(CPUTIMER0_BASE);

    for(;;)
    {
        Hall_update();
        Log_service();

        // LED2 on = fault
        GPIO_writePin(DEVICE_GPIO_PIN_LED2,
                      (motorState == BLDC_FAULT) ? 0U : 1U);
    }
}

__interrupt void cpuTimer0ISR(void)
{
    GPIO_togglePin(DEVICE_GPIO_PIN_LED1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
