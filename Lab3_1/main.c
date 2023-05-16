
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

#include "Adafruit_SSD1351.h"
#include "Adafruit_GFX.h"
#include "glcdfont.h"
#include "spi.h"

// Common interface includes
#include "uart_if.h"
#include "uart.h"

// Pin configurations
#include "pin_mux_config.h"

// Signal Bits
#define B0      0b00000010111111010000000011111111
#define B1      0b00000010111111011000000001111111
#define B2      0b00000010111111010100000010111111
#define B3      0b00000010111111011100000000111111
#define B4      0b00000010111111010010000011011111
#define B5      0b00000010111111011010000001011111
#define B6      0b00000010111111010110000010011111
#define B7      0b00000010111111011110000000011111
#define B8      0b00000010111111010001000011101111
#define B9      0b00000010111111011001000001101111
#define MUTE    0b00000010111111010000100011110111
#define LAST    0b00000010111111010000001011111101

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#define SPI_IF_BIT_RATE  100000

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
volatile unsigned SW_intflag;
volatile int first_edge = 1;
int start  = 0;
char  prevLetter = '/';
char buffer[64];
int bufIndex = 0;

volatile long currButton;
volatile long prevButton;
volatile long prevData;
int sameButton = 0;
int EnterMessage = 0;

unsigned long data = 0;
int track = 0;
int color = 1;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

// an example of how you can use structs to organize your pin settings for easier maintenance
typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting button = { .port = GPIOA0_BASE, .pin = 0x80};

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************

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

// Depicts the Color of the Text
void DisplayColor(void)
{

    if(color == 5)
        color = 1;
    else
        color++;

    switch(color)
    {
        case 1:
            Report("Color is now: WHITE \n\r");
            break;
        case 2:
            Report("Color is now: BLUE \n\r");
            break;
        case 3:
            Report("Color is now: GREEN \n\r");
            break;
        case 4:
            Report("Color is now: CYAN \n\r");
            break;
        case 5:
            Report("Color is now: RED \n\r");
            break;
        default:
            break;
    }
}

// Standard Letter of Each Button
char firstLetter(unsigned long value)
{
    char letter;
    switch(value)
    {
        case B0:
            letter = ' ';
            Report("letter: %c \n\r", letter);
            break;
        case B1:
            letter = '*';
            DisplayColor();
            break;
        case B2:
            letter = 'a';
            Report("letter: %c \n\r", letter);
            break;
        case B3:
            letter = 'd';
            Report("letter: %c \n\r", letter);
            break;
        case B4:
            letter = 'g';
            Report("letter: %c \n\r", letter);
            break;
        case B5:
            letter = 'j';
            Report("letter: %c \n\r", letter);
            break;
        case B6:
            letter = 'm';
            Report("letter: %c \n\r", letter);
            break;
        case B7:
            letter = 'p';
            Report("letter: %c \n\r", letter);
            break;
        case B8:
            letter = 't';
            Report("letter: %c \n\r", letter);
            break;
        case B9:
            letter = 'w';
            Report("letter: %c \n\r", letter);
            break;
        case MUTE:
            letter = '-';
            Report("letter: %c \n\r", letter);
            break;
        case LAST:
            letter = '+';
            Report("letter: %c \n\r", letter);
            break;
        default:
            Report("error not valid. \n\r", letter);
            break;
    }
    return letter;
}

// Helps the Button Presses Iterate
char DisplayNextLetter(char l)
{
    char letter;
    switch(l)
    {
        case ' ':
            letter = ' ';
            Report("letter: %c \n\r", letter);
            break;
        case '*':
            letter = '*';
            DisplayColor();
            break;
        case 'a':
            letter = 'b';
            Report("letter: %c \n\r", letter);
            break;
        case 'b':
            letter = 'c';
            Report("letter: %c \n\r", letter);
            break;
        case 'c':
            letter = 'a';
            Report("letter: %c \n\r", letter);
            break;
        case 'd':
            letter = 'e';
            Report("letter: %c \n\r", letter);
            break;
        case 'e':
            letter = 'f';
            Report("letter: %c \n\r", letter);
            break;
        case 'f':
            letter = 'd';
            Report("letter: %c \n\r", letter);
            break;
        case 'g':
            letter = 'h';
            Report("letter: %c \n\r", letter);
            break;
        case 'h':
            letter = 'i';
            Report("letter: %c \n\r", letter);
            break;
        case 'i':
            letter = 'g';
            Report("letter: %c \n\r", letter);
            break;
        case 'j':
            letter = 'k';
            Report("letter: %c \n\r", letter);
            break;
        case 'k':
            letter = 'l';
            Report("letter: %c \n\r", letter);
            break;
        case 'l':
            letter = 'j';
            Report("letter: %c \n\r", letter);
            break;
        case 'm':
            letter = 'n';
            Report("letter: %c \n\r", letter);
            break;
        case 'n':
            letter = 'o';
            Report("letter: %c \n\r", letter);
            break;
        case 'o':
            letter = 'm';
            Report("letter: %c \n\r", letter);
            break;
        case 'p':
            letter = 'q';
            Report("letter: %c \n\r", letter);
            break;
        case 'q':
            letter = 'r';
            Report("letter: %c \n\r", letter);
            break;
        case 'r':
            letter = 's';
            Report("letter: %c \n\r", letter);
            break;
        case 's':
            letter = 'p';
            Report("letter: %c \n\r", letter);
            break;
        case 't':
            letter = 'u';
            Report("letter: %c \n\r", letter);
            break;
        case 'u':
            letter = 'v';
            Report("letter: %c \n\r", letter);
            break;
        case 'v':
            letter = 't';
            Report("letter: %c \n\r", letter);
            break;
        case 'w':
            letter = 'x';
            Report("letter: %c \n\r", letter);
            break;
        case 'x':
            letter = 'y';
            Report("letter: %c \n\r", letter);
            break;
        case 'y':
            letter = 'z';
            Report("letter: %c \n\r", letter);
            break;
        case 'z':
            letter = 'w';
            Report("letter: %c \n\r", letter);
            break;
        case '-':
            letter = '-';
            Report("letter: %c \n\r", letter);
            break;
        case '+':
            letter = '+';
            Report("letter: %c \n\r", letter);
            break;
        default:
            Report("error\n\r");
            break;
    }
    return letter;
}

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


// UART Handler
void UARTIntHandler(void)
{
    // Checks Interrupt Status
    if(0)
    {
        //Clears Interrupt

        while(UARTCharsAvail(UARTA1_BASE))
        {
            // Collects Data
            long buffer = UARTCharGetNonBlocking(UARTA1_BASE);
        }
    }
}

// Sets Up UART Communication
void UART_Communication(void)
{
    MAP_UARTConfigSetExpClk(UARTA1_BASE, SYSCLK, UART_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFODisable(UARTA1_BASE);
    MAP_UARTIntRegister(UARTA1_BASE, UARTIntHandler);
    UARTFIFOLevelSet(UARTA1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    unsigned long ulStatus;
    ulStatus = MAP_UARTIntStatus(UARTA1_BASE, false);
    MAP_UARTIntClear(UARTA1_BASE, ulStatus);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX);
}

// Sets Up SPI Comm
static void SPI_Communication(void){

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();
//    delay(100);
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

        if (delta_us >= 35000)
        {
            SW_intcount = 0;
        }else if (delta_us >= 2500 && delta_us < 35000)
        { // Finds Start Time
            data = 0;
            SW_intcount = 1;
        }if(delta_us > 1300 && delta_us < 2500)
        {// Determines 1 bit
            data = data << 1;
            data = data + 1;
        } else if(delta_us < 1300)
        {// Determines 0 bit
            data = data << 1;
        }else if(delta_us > 2500)
        {
            data = 0;
        }
    }

    SW_intcount++;

    // Resets interrupt handle for next button pressed
    if (SW_intcount == 34) {
        SW_intcount = 0;
        first_edge = 1;
        SW_intflag = 1;
    }

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus (button.port, true);
    MAP_GPIOIntClear(button.port, ulStatus);       // clear interrupts on GPIOA2

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
    
    UART_Communication();

    // Enable SysTick
    SysTickInit();

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    // Set SPI
    SPI_Communication();

    //
    // Register the interrupt handlers
    //
    MAP_GPIOIntRegister(button.port, GPIOA2IntHandler);

    //
    // Configure rising edge interrupts on SW2 and SW3
    //

    MAP_GPIOIntTypeSet(button.port, button.pin, GPIO_FALLING_EDGE);    // SW2

    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(button.port, false);
    MAP_GPIOIntClear(button.port, ulStatus);           // clear interrupts on GPIOA2

    // clear global variables
    SW_intcount=0;
    SW_intflag=0;

    //int col = 0;

    // Enable SW2 and SW3 interrupts
    MAP_GPIOIntEnable(button.port, button.pin);

    Message("\t\t****************************************************\n\r");
    Message("\t\t\t\tSystick Example\n\r\n\r");
    Message("\t\t to delete press MUTE button\n\r");
    Message("\t\t to enter press LAST button\n\r");
    Message("\t\t****************************************************\n\r");
    Message("\n\n\n\r");

    prevData = 1;
    currButton = -2;
    prevButton = -1;
    char letter;

    while (1) {
        while(SW_intflag == 0){;}
        DisplayButtonPressed(data);
        prevData = data;
        letter = firstLetter(prevData);
        SW_intflag = 0;

        if (prevData != B0 && prevData != B1 && prevData != MUTE && prevData != LAST) {
            uint64_t timeInterval = 0;
            while (timeInterval++ < 3500000) {
                if (SW_intflag) {
                    // Determines if its the same button
                    if(prevData == data)
                    {
                        sameButton = 1;
                        currButton++;
                    }
                    else
                        sameButton = 0;

                    // Displays Letter
                    if(sameButton)
                    {
                        letter = DisplayNextLetter(letter);
                        timeInterval = 0;
                    }
                    else
                    {
                        SW_intflag = 1;
                        break;
                    }
                    SW_intflag = 0;
                }
            }
        }

        // Returns the Button Selected if there are Consecutive Presses
        if(letter != '*')
            Report("letter %c selected \n\r", letter);

        // Prints full String
        if (letter == '+') {
            Report("String: %s \n\r", buffer);

            // Sends Buffer Through UART
            int i=0;
            for(i = 0; i < bufIndex; i++)
            {
                //if(buffer[i] != '*')
                  //  Report("String: %c \n\r", buffer[i]);

                MAP_UARTCharPut(UARTA1_BASE, buffer[i]);
            }
            bufIndex = 0;
            MAP_UARTCharPut(UARTA1_BASE, '=');
            // Resets Buffer
            for (i = 0; i < 64; i++) {
                buffer[i] = '\0';
            }

        } // Deletes Last Letter
        else if (letter == '-') {
            if (bufIndex > 0) {
                buffer[--bufIndex] = '\0';
                buffer[--bufIndex] = '\0';
            }
        } // Sets New Letter
        else {
            if(letter != '*')
            {
                buffer[bufIndex++] = letter;
                buffer[bufIndex++] = color;
            }
        }

        // Saves New Button Information
        prevButton = currButton;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
