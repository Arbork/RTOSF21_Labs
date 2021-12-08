#ifndef SRC_HEADER_FILES_GPIO_H_
#define SRC_HEADER_FILES_GPIO_H_

//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
#include "bspconfig.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED 0 pin is
#define LED0_port   gpioPortF
#define LED0_pin    4u
#define LED0_default  false   // Default false (0) = off, true (1) = on
// LED 1 pin is
#define LED1_port   gpioPortF
#define LED1_pin    5u
#define LED1_default  false // Default false (0) = off, true (1) = on

#define BTN0_port       gpioPortF
#define BTN0_pin        6u
#define BTN1_port       gpioPortF
#define BTN1_pin        7u
#define BTN0_default    false
#define BTN1_default    false
#define BTN_mask        0x00000003

#define CAP_mask        0x0000000c

#define ODD_GPIO_IRPT   0xAAAA
#define EVEN_GPIO_IRPT  0x5555

//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_open(void);


#endif /* SRC_HEADER_FILES_GPIO_H_ */
