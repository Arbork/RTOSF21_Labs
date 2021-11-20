//***********************************************************************************
// Include files
//***********************************************************************************
#include "gpio.h"

//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************


//***********************************************************************************
// functions
//***********************************************************************************

/***************************************************************************//**
 * @brief
 * configures the gpio pins
 *
 * @details
 * sets up drivestrength and pin modes for all in use gpios
 * these include the leds and the buttons
 *
 * @note
 * if any other gpio even interrupt is triggered we want to assert false because none of them should even be enabled
 *
 ******************************************************************************/
void gpio_open(void){
    // Set LED ports to be standard output drive with default off (cleared)
    GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
    //  GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, LED0_default);

    GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
    //  GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
    GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, LED1_default);

    GPIO_ExtIntConfig(BTN0_port, BTN0_pin, BTN0_pin, false, true, true); // Enable both posedge and negedge interrupts for PB1 and PB0
    GPIO_ExtIntConfig(BTN1_port, BTN1_pin, BTN1_pin, false, true, true);

    NVIC_EnableIRQ(GPIO_EVEN_IRQn); //Enable even and odd pin interrupts for the gpio pins that pb0 and 1 are on
    NVIC_EnableIRQ(GPIO_ODD_IRQn);

}
