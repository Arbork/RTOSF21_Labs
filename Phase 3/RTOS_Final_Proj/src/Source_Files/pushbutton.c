/* @file    pushbutton.c
 * @author  Alexander Bork
 * @date    9/7/2021
 * @brief   Contains functions to pole, store, and return the state of the pushbuttons
 */

//***********************************************************************************
// Include files
//***********************************************************************************
#include "pushbutton.h"
#include "os.h"
//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// global variables
//***********************************************************************************
static uint8_t PB0_STATE;   // Integers holding the state of each pushbutton separately
static uint8_t PB1_STATE;

//***********************************************************************************
// function
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *      Initialize the states of the PB variables.
 *
 * @details
 *      Both PB0 and PB1 will be set to 1, as that is the active low for the pushbuttons.
 *
 * @note
 *      Is called once from app.c to setup the Button's states.
 ******************************************************************************/
void initButtonSM(void){
    PB0_STATE = 1;
    PB1_STATE = 1;
}

/***************************************************************************//**
 * @brief
 *      Function that returns the state of pushbutton0 state variable.
 *
 * @details
 *      PB0_STATE is anded with the PB_MASK to ensure that it only returns a
 *      1-bit integer containing the state of PB0.
 *
 * @return
 *      An 8-bit integer where only bit position 0 does not get masked away.
 *
 ******************************************************************************/
uint8_t get_buttonState0(void){
    return PB0_STATE & ~PB_MASK;
}

/***************************************************************************//**
 * @brief
 *      Function that returns the state of pushbutton1 state variable.
 *
 * @details
 *      PB1_STATE is anded with the PB_MASK to ensure that it only returns a
 *      1-bit integer containing the state of PB1.
 *
 * @return
 *      An 8-bit integer where only bit position 0 does not get masked away.
 *
 ******************************************************************************/
uint8_t get_buttonState1(void){
    return PB1_STATE & ~PB_MASK;
}

/***************************************************************************//**
 * @brief
 *      Function to update the state variable of PB0.
 *
 * @details
 *      PB0_STATE is populated with the status of GPIO portF pin 6. 1 being unpressed
 *      and 0 being pressed.
 *
 * @note
 *      The function does not return anything.
 ******************************************************************************/
void update_button0State(void){
    PB0_STATE = GPIO_PinInGet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN); // Read portF pin 6
}

/***************************************************************************//**
 * @brief
 *      Function to update the state variable of PB1.
 *
 * @details
 *      PB1_STATE is populated with the status of GPIO portF pin 6. 1 being unpressed
 *      and 0 being pressed.
 *
 * @note
 *      The function does not return anything.
 ******************************************************************************/
void update_button1State(void){
    PB1_STATE = GPIO_PinInGet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN); // Read portF pin 7
}







