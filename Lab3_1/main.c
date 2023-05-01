
//*****************************************************************************
//
// Application Name     - TV Remote Decoder (TV Code: Zonda 1355)
// Application Overview - The objective of this application is to demonstrate
//                          GPIO interrupts using SW2 and SW3.
//                          NOTE: the switches are not debounced!
//
//*****************************************************************************

// Standard includes
#include <stdio.h>
#include <stdint.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "prcm.h"
#include "utils.h"
#include "systick.h"
#include "rom_map.h"
#include "interrupt.h"
#include "gpio.h"
#include "utils.h"

// Common interface includes
#include "uart_if.h"

// Pin configurations
#include "pin_mux_config.h"

/*
#define B0      0xB00000010111111010000000011111111
#define B1      0x2FD807F // 0000 0010 1111 1101 1000 0000 0111 1111
#define B2      0x2FD40BF // 0000 0010 1111 1101 0100 0000 1011 1111
#define B3      0x2FDC03F // 0000 0010 1111 1101 1100 0000 0011 1111
#define B4      0x2FD20DF // 0000 0010 1111 1101 0010 0000 1101 1111
#define B5      0x2FDA05F // 0000 0010 1111 1101 1010 0000 0101 1111
#define B6      0x2FD609F // 0000 0010 1111 1101 0110 0000 1001 1111
#define B7      0x2FD //
#define B8      0x2FD
#define B9      0x2FD
#define MUTE    0x2FD
#define LAST    0x2FD*/

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL
#define ZERO_INT 100

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);

volatile unsigned long SW_intcount = 0;
volatile unsigned char SW_intflag;
volatile uint64_t store[100];
volatile int storeCount =0;
volatile int first_edge = 1;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting button = { .port = GPIOA2_BASE, .pin = 0x2};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

// Displays Message According to the Button Pressed
/*void DisplayButtonPressed(unsigned long value)
{
    switch(value)
    {
        case B0:
            Report("0 was pressed. \n\r");
            break;
 /*       case B1:
            Report("1 was pressed. \n\r");
            break;
        case B2:
            Report("2 was pressed. \n\r");
            break;
        case B3:
            Report("3 was pressed. \n\r");
            break;
        case B4:
            Report("4 was pressed. \n\r");
            break;
        case B5:
            Report("5 was pressed. \n\r");
            break;
        case B6:
            Report("6 was pressed. \n\r");
            break;
        case B7:
            Report("7 was pressed. \n\r");
            break;
        case B8:
            Report("8 was pressed. \n\r");
            break;
        case B9:
            Report("9 was pressed. \n\r");
            break;
        case LAST:
            Report("Last was pressed. \n\r");
            break;
        case MUTE:
            Report("Mute was pressed. \n\r");
            break;
        default:
            Report("Error. \n\r");
            break;
    }
}*/

/**
 * Reset SysTick Counter
 */
static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
    systick_cnt = 0;
}

/**
 * SysTick Interrupt Handler
 *
 * Keep track of whether the systick counter wrapped
 */
static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);

    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

/**
 * Initializes SysTick Module
 */
static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}

static void GPIOA2IntHandler(void) {    // SW2 handler

    if (first_edge) {
        SysTickReset();
        first_edge = 0;
    }
    else {
        // read the countdown register and compute elapsed cycles
        uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();

        // convert elapsed cycles to microseconds
        uint64_t delta_us = TICKS_TO_US(delta);

        SysTickReset();

        // print measured time to UART
        //Report("cycles = %d\tms = %d\tedgecount = %d\n\r", delta, delta_us, SW_intcount);

        store[storeCount] = delta_us;

        if(storeCount < 100-1)
            storeCount++;
        else
            storeCount = 0;

    }
    SW_intcount++;
    if (SW_intcount == 34) {
        SW_intcount = 0;
        first_edge = 1;
        storeCount = 0;
        SW_intflag = 1;
    }

    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (button.port, true);
    MAP_GPIOIntClear(button.port, ulStatus);       // clear interrupts on GPIOA2

    /*
    SW_intflag=1;

    SW_intflag = 0;
    */

}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {

    BoardInit();
    
    PinMuxConfig();
    
    // Enable SysTick
    SysTickInit();

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(button.port, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //

    MAP_GPIOIntTypeSet(button.port, button.pin, GPIO_BOTH_EDGES);    // SW2

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (button.port, false);
    MAP_GPIOIntClear(button.port, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW_intcount=0;
    SW_intflag=0;

    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(button.port, button.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\tSystick Example\n\r");
    Message("\t\t ****************************************************\n\r");
    Message("\n\n\n\r");

    while (1) {
        while(SW_intflag == 0){;}

/*        SW_intflag = 0;
        // reset the countdown register
        SysTickReset();

        // wait for a fixed number of cycles
        // should be 3000 i think (see utils.c)
        UtilsDelay(1000);

        // read the countdown register and compute elapsed cycles
        uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();

        // convert elapsed cycles to microseconds
        uint64_t delta_us = TICKS_TO_US(delta);



        // print measured time to UART
        Report("cycles = %d\tms = %d\n\r", delta, delta_us); */
        /*
        Report("SW2 ints = %d\r\n",SW_intcount);
        UtilsDelay(3000000);
        */
        int i;
        for (i = 0; i < 33; i++) {
            Report("ms = %d\tedgecount = %d\n\r", store[i], i);
        }
        SW_intflag = 0;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
