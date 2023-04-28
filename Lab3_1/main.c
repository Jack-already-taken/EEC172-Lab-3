// Jack Xiang
// Anayeli Martinez 

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include <stdio.h>
#include <stdint.h>

#include "hw_nvic.h"
#include "hw_apps_rcm.h"
#include "systick.h"


// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"


#define APPLICATION_VERSION     "1.1.1"
#define B0      0x2FD
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
#define LAST    0x2FD
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

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;
/*
extern void (* const g_pfnVectors[])(void);


#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif*/


//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

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

static void GPIOA1IntHandler(void) { // SW3 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA2_BASE, true); // Use GPIO Pin Input (aka IR output)
    MAP_GPIOIntClear(GPIOA2_BASE, ulStatus);          // clear interrupts on GPIOA2
//    SW3_intcount++;

//    SW3_intflag=1;
}

// Displays Message According to the Button Pressed
void DisplayButtonPressed(unsigned long value)
{
    switch(value)
    {
        case B0:
            Report("0 was pressed. \n\r");
            break;
        case B1:
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
            break;
    }
}
//*****************************************************************************
//
// Board Initialization & Configuration
//
// \param  None
//
// \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
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

static void GPIO_Interrupt()
{
    // Register Interrupt Handler
    MAP_GPIOIntRegister(GPIOA2_BASE, FallingEdge()); // recommend falling edge
    // Configure Particular Edge
    MAP_GPIOIntTypeSet(GPIOA2_BASE, 0x40, GPIO_FALLING_EDGE);
    // Clear Interrupt
    MAP_GPIOIntClear(GPIOA2_BASE, MAP_GPIOIntStatus(GPIOA2_BASE, true));
    // Enable Interrupt 
    MAP_GPIOIntEnable(GPIOA2_BASE, 0x40);


}

static void FallingEdge()
{
    
   
}
//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    // Enable SysTick
    SysTickInit();

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t        CC3200 IR Transmission Application  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
   // MAP_PRCMPeripheralReset(PRCM_GSPI);
// GPIO = determines (falling/rising edge)pulses that are coming
// Systic = reload value - just define it, it gets called automatically

    GPIO_Interrupt();

    while (1) {
        // reset the countdown register
        SysTickReset();

        // wait for a fixed number of cycles
        // should be 3000 i think (see utils.c)
        UtilsDelay(1000);

        // read the countdown register and compute elapsed cycles
        uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();
        //total ticks - gets current number of ticks = completed number of ticks

        // convert elapsed cycles to microseconds
        uint64_t delta_us = TICKS_TO_US(delta); // Determines whether it is 0 or 1

        // print measured time to UART
        Report("cycles = %d\tms = %d\n\r", delta, delta_us); // 3 byte = data
        // Clear GPIO interrupt

    }

}

/*    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);*/
