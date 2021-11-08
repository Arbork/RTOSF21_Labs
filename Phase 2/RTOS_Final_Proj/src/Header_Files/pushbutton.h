/*
 * button.h
 *
 *  Created on: Sep 1, 2021
 *      Author: Alexander Bork
 */

#ifndef HEADER_FILES_PUSHBUTTON_H_
#define HEADER_FILES_PUSHBUTTON_H_
//***********************************************************************************
// Include files
//***********************************************************************************

#include "em_gpio.h"
#include "bspconfig.h"
//***********************************************************************************
// defined files
//***********************************************************************************

//***********************************************************************************
// global variables
//***********************************************************************************
#define PB0     0
#define PB1     1
#define PB_MASK 0xfffffffe
//***********************************************************************************
// function prototypes
//***********************************************************************************
uint8_t get_buttonState0(void);
uint8_t get_buttonState1(void);
void initButtonSM(void);
void update_button0State(void);
void update_button1State(void);


#endif /* HEADER_FILES_PUSHBUTTON_H_ */

