/* @file    task.c
 * @author  Alexander Bork
 * @date    9/16/2021
 * @brief   Creates tasks for the OS and defines functionality
 */
#include "task.h"

// Private functions for the 9 Tasks
static void physEng_task(void * arg);
static OS_TCB PhysEng;
static CPU_STK stack1[PHYS_TASK_STACK_SIZE];

static void landing_task(void * arg);
static OS_TCB LAND;
static CPU_STK stack2[LAND_TASK_STACK_SIZE];

static void LCD_task(void * arg);
static OS_TCB LCD;
static CPU_STK stack3[LCD_TASK_STACK_SIZE];

static void fuel_task(void * arg);
static OS_TCB FUEL;
static CPU_STK stack4[FUEL_TASK_STACK_SIZE];

static void AOA_task(void * arg);
static OS_TCB AOA;
static CPU_STK stack5[AOA_TASK_STACK_SIZE];

static void postgame_task(void * arg);
static OS_TCB PG;
static CPU_STK stack6[PG_TASK_STACK_SIZE];

static void LEDON_task(void * arg);
static OS_TCB LEDON;
static CPU_STK stack7[LEDON_TASK_STACK_SIZE];

static void LEDOFF_task(void * arg);
static OS_TCB LEDOFF;
static CPU_STK stack8[LEDOFF_TASK_STACK_SIZE];


static void idle_task(void *arg);
static OS_TCB IDLE;
static CPU_STK stack9[IDLE_TASK_STACK_SIZE];


static volatile struct node_t * queueHead;




///////////////////////////////////////////////////////////////////////////
///
/// Data structures
///
/// //////////////////////////////////////////////////////////////////////

static struct SSData appSP;
static struct VDData appVD;


/////////////////////////////////////////////////////////////////////

/*******************************************************************************
 * @brief
 *      Function to create tasks and resource managers that the OS will use
 *
 * @details
 *      There are five tasks created in the function: Speed Setpoint, Vehicle Direction,
 *      Vehicle Monitor, LED Output, and the IDLE task. And there are 3 mutexes, 2 flags,
 *      and one semaphore created as well.
 *
 * @note
 *      LED task has the highest priority, then speed setpoint, then vehicle direction, then vehicle Monitor,
 *      then IDLE.
 *      Stack size and priority are defined in task.h.
 *
 ******************************************************************************/
void task_init(void){
      RTOS_ERR err;

      // Create LED task
      OSTaskCreate(&PhysEng,
               "Physics Engine",
               physEng_task,
               DEF_NULL,
               PHYS_TASK_PRIO,
               &stack1[0],
               (PHYS_TASK_STACK_SIZE / 10u),
               PHYS_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      // Create button task
      OSTaskCreate(&LAND,
               "Landing Task",
               landing_task,
               DEF_NULL,
               LAND_TASK_PRIO,
               &stack2[0],
               (LAND_TASK_STACK_SIZE / 10u),
               LAND_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&LCD,
               "LCD Task",
               LCD_task,
               DEF_NULL,
               LCD_TASK_PRIO,
               &stack3[0],
               (LCD_TASK_STACK_SIZE / 10u),
               LCD_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&FUEL,
               "Fuel Task",
               fuel_task,
               DEF_NULL,
               FUEL_TASK_PRIO,
               &stack4[0],
               (FUEL_TASK_STACK_SIZE / 10u),
               FUEL_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&AOA,
               "AOA Task",
               AOA_task,
               DEF_NULL,
               AOA_TASK_PRIO,
               &stack5[0],
               (AOA_TASK_STACK_SIZE / 10u),
               AOA_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&PG,
               "Post Game Task",
               postgame_task,
               DEF_NULL,
               PG_TASK_PRIO,
               &stack6[0],
               (PG_TASK_STACK_SIZE / 10u),
               PG_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&LEDON,
               "LED ON",
               LEDON_task,
               DEF_NULL,
               FUEL_TASK_PRIO,
               &stack7[0],
               (LEDON_TASK_STACK_SIZE / 10u),
               LEDON_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&LEDOFF,
               "LED OFF",
               LEDOFF_task,
               DEF_NULL,
               LEDOFF_TASK_PRIO,
               &stack4[0],
               (LEDOFF_TASK_STACK_SIZE / 10u),
               LEDOFF_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

    //Create the idle task
   OSTaskCreate(&IDLE,
             "Idle Task",
             idle_task,
             DEF_NULL,
             IDLE_TASK_PRIO,
             &stack9[0],
             (IDLE_TASK_STACK_SIZE / 10u),
             IDLE_TASK_STACK_SIZE,
             0u,
             0u,
             DEF_NULL,
             (OS_OPT_TASK_STK_CLR),
             &err);
   EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      ///////////////////////////////////////////////
      /// Create resources
      /// ///////////////////////////////////////////

}



/*******************************************************************************
 * @brief
 *      Function that defines the LED ON task
 *
 * @details
 *      The LED pins are initialized before the while loop. And the LED_task will update
 *      the LEDs based off of violations sent to it via a flag post from the vehicle monitor
 *      task.
 *
 * @note
 *      This task will stay blocked as long as it does not have a flag posted to it via
 *      LED_Flag.
 *
 ******************************************************************************/
static void LEDON_task(void *arg){
    // Drive the leds based on input
    RTOS_ERR  err;
    OS_FLAGS  flags;
    while(1){

    }
}

/*******************************************************************************
 * @brief
 *      Function that defines the LED OFF task
 *
 * @details
 *      The LED pins are initialized before the while loop. And the LED_task will update
 *      the LEDs based off of violations sent to it via a flag post from the vehicle monitor
 *      task.
 *
 * @note
 *      This task will stay blocked as long as it does not have a flag posted to it via
 *      LED_Flag.
 *
 ******************************************************************************/
static void LEDOFF_task(void *arg){
    // Drive the leds based on input
    RTOS_ERR  err;
    OS_FLAGS  flags;
    while(1){

    }
}


/***************************************************************************//**
 * @brief
 *      Interrupt Handler for even GPIO interrupts
 *
 * @details
 *      All even interrupts will be cleared from the GPIO Int flag register.
 *      This function will then request the state of pushButton0 and update the
 *      global variable within pushbutton.c. Both negedge and posedge interrupts
 *      are enabled.
 *
 * @note
 *      This interrupt is trigger for PB0, as it is on PortF - pin 6
 *
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void){
    // Clear all even pin interrupt flags
    GPIO_IntClear(EVEN_GPIO_IRPT);
    RTOS_ERR err;
    OS_SEM_CTR  ctr;
    update_button0State();
    update_button1State();
    uint8_t buttonSt = (get_buttonState1() << 1) | get_buttonState0();
    if(buttonSt == 3){
        push(&queueHead, buttonNone);
    }
    else if(buttonSt == 2){
        push(&queueHead, button0);
    }
    else if(buttonSt == 1){
        push(&queueHead, button1);
    }
    else{
        push(&queueHead, buttonBoth);
    }
    ctr = OSSemPost(&APP_Sema,    /* Pointer to user-allocated semaphore.    */
                    OS_OPT_POST_ALL,    /* Only wake up highest-priority task.     */
                    &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

}


/***************************************************************************//**
 * @brief
 *      Interrupt Handler for odd GPIO interrupts
 *
 * @details
 *      All odd interrupts will be cleared from the GPIO Int flag register.
 *      This function will then request the state of pushButton1 and update the
 *      global variable within pushbutton.c. Only the negedge interrupts
 *      are enabled and valid.
 *
 * @note
 *      This interrupt is trigger for PB1, as it is on PortF - pin 7
 *
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void){
    // Clear all even pin interrupt flags
    GPIO_IntClear(ODD_GPIO_IRPT);
    RTOS_ERR err;
    OS_SEM_CTR  ctr;
    update_button0State();
    update_button1State();
    uint8_t buttonSt = (get_buttonState1() << 1) | get_buttonState0();
    if(buttonSt == 3){
        push(&queueHead, buttonNone);
    }
    else if(buttonSt == 2){
        push(&queueHead, button0);
    }
    else if(buttonSt == 1){
        push(&queueHead, button1);
    }
    else{
        push(&queueHead, buttonBoth);
    }

    ctr = OSSemPost(&APP_Sema,    /* Pointer to user-allocated semaphore.    */
                    OS_OPT_POST_ALL,    /* Only wake up highest-priority task.     */
                    &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

}



/*******************************************************************************
 * @brief
 *      Function that defines the idle task
 *
 * @details
 *      This function only keeps the board in energy mode 1.
 *
 * @note
 *      An OS delay of 5ms is used.
 *
 ******************************************************************************/
static void idle_task(void *arg)
{
//    RTOS_ERR err;
    // Read the capsensor
    while(1){
        //EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        EMU_EnterEM1(); // Go into energy mode 1
    }
}


