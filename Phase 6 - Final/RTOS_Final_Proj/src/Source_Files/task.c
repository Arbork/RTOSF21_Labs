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

static void LEDON0_task(void * arg);
static OS_TCB LEDON0;
static CPU_STK stack7[LEDON_TASK_STACK_SIZE];

static void LEDOFF0_task(void * arg);
static OS_TCB LEDOFF0;
static CPU_STK stack8[LEDOFF_TASK_STACK_SIZE];

static void LEDON1_task(void * arg);
static OS_TCB LEDON1;
static CPU_STK stack10[LEDON_TASK_STACK_SIZE];

static void LEDOFF1_task(void * arg);
static OS_TCB LEDOFF1;
static CPU_STK stack11[LEDOFF_TASK_STACK_SIZE];


static void idle_task(void *arg);
static OS_TCB IDLE;
static CPU_STK stack9[IDLE_TASK_STACK_SIZE];


//static volatile struct node_t * queueHead;




///////////////////////////////////////////////////////////////////////////
///
/// Data structures
///
/// //////////////////////////////////////////////////////////////////////

static volatile struct DutyCycle DC0;
static volatile struct DutyCycle DC1;

static volatile struct FuelStr fuelDat;
static volatile struct AngleStr AngStr;
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

      OSTaskCreate(&LEDON0,
               "LED0 ON",
               LEDON0_task,
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

      OSTaskCreate(&LEDOFF0,
               "LED0 OFF",
               LEDOFF0_task,
               DEF_NULL,
               LEDOFF_TASK_PRIO,
               &stack8[0],
               (LEDOFF_TASK_STACK_SIZE / 10u),
               LEDOFF_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

      OSTaskCreate(&LEDON1,
                     "LED1 ON",
                     LEDON1_task,
                     DEF_NULL,
                     FUEL_TASK_PRIO,
                     &stack10[0],
                     (LEDON_TASK_STACK_SIZE / 10u),
                     LEDON_TASK_STACK_SIZE,
                     0u,
                     0u,
                     DEF_NULL,
                     (OS_OPT_TASK_STK_CLR),
                     &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE)); // Check that there are no errors in creating the task.

            OSTaskCreate(&LEDOFF1,
                     "LED1 OFF",
                     LEDOFF1_task,
                     DEF_NULL,
                     LEDOFF_TASK_PRIO,
                     &stack11[0],
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
}



static void physEng_task(void * arg){
    RTOS_ERR err;

    while(1){
        OSTimeDly(FUEL_POLL_FREQ, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
    }
}



static void landing_task(void * arg){
    while(1){

    }
}



static void LCD_task(void * arg){
    while(1){

    }
}



static void fuel_task(void * arg){
    RTOS_ERR err;
    uint8_t buttonSt = 3;
    uint32_t thrustPcnt = 0;
    uint32_t rate = 0;
    uint8_t updated = 0;
    while(1){
        OSTimeDly(FUEL_POLL_FREQ, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        update_button0State();
        update_button1State();
        buttonSt = (get_buttonState1() << 1) | get_buttonState0();
        if(buttonSt == 1){      // If button 1 is pressed
            if(thrustPcnt <= 90){
                thrustPcnt += THRUST_QUANTA;
                rate = (thrustPcnt * MAXTHRUST);
                updated = 1;
            }

        }
        else if(buttonSt == 2){     // If button 0 is pressed
            if(thrustPcnt >= 10){
                thrustPcnt -= THRUST_QUANTA;
                rate = (thrustPcnt * MAXTHRUST);
                updated = 1;
            }
        }
        if(updated){
            // Acquire mutex on fuel data struct and update the fields
            OSMutexPend(&FUEL_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            fuelDat.thrustPrct = thrustPcnt;
            fuelDat.consRate = rate;

            OSMutexPost(&FUEL_MUTEX, OS_OPT_POST_ALL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            // Acquire mutex on fuel data struct and update the duty cycle
            OSMutexPend(&DUTYCYC0_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            DC0.dutyCycle = thrustPcnt;

            OSMutexPost(&DUTYCYC0_MUTEX, OS_OPT_POST_ALL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
            updated = 0;
        }
    }
}



static void AOA_task(void * arg){
    RTOS_ERR err;
    uint8_t capState = 15;
    uint8_t updated = 0;
    int8_t angleDelta = 0;
    int8_t prevAngle = 0;
    while(1){
        OSTimeDly(AOA_POLL_FREQ, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        CAPSENSE_Sense();       // Sense the capsense
        capState = CAPSENSE_getPressed(CHANNEL3) << 3;      // Set bit 4 low if the far left pad is being touched
        capState |= CAPSENSE_getPressed(CHANNEL2) << 2;     // Set bit 3 low if the middle left pad is being touched
        capState |= CAPSENSE_getPressed(CHANNEL1) << 1;     // Set bit 2 low if the middle right pad is being touched
        capState |= CAPSENSE_getPressed(CHANNEL0);          // Set bit 1 low if the far right pad is being touched

        switch(capState){
            case 15:    // 0b1111
            case 9:     // 0b1001
            case 6:     // 0b0110
            case 0:     // 0b0000
                angleDelta = 0;
                break;
            case 14:    // 0b1110
                angleDelta = RIGHT_MOST_PAD;
                updated = 1;
                break;
            case 7:     // 0b0111
                angleDelta = LEFT_MOST_PAD;
                updated = 1;
                break;
            case 13:                            // 0b1101
                angleDelta = RIGHT_INNER_PAD;
                updated = 1;
                break;
            case 11:                            // 0b1011
                angleDelta = LEFT_INNER_PAD;
                updated = 1;
                break;
            case 3:                             // 0b0011
                angleDelta = LEFT_MOST_PAD;
                updated = 1;
                break;
            case 12:                            // 0b1100
                angleDelta = RIGHT_MOST_PAD;
                updated = 1;
                break;
            case 8:
                angleDelta = RIGHT_MOST_PAD;   // 0b1000
                updated = 1;
                break;
            case 4:
                angleDelta = RIGHT_INNER_PAD;   // 0b0100
                updated = 1;
                break;
            case 1:                             // 0b0001
                angleDelta = LEFT_MOST_PAD;
                updated = 1;
                break;
            case 2:         // 0b0010
                angleDelta = LEFT_INNER_PAD;
                updated = 1;
                break;
            case 5:         // 0b0101
                angleDelta = LEFT_INNER_PAD;
                updated = 1;
                break;
            case 10:        // 0b1010
                angleDelta = RIGHT_INNER_PAD;
                updated = 1;
                break;
            default:
                EFM_ASSERT(false);
                break;
        }
        if(updated){
            // Acquire mutex on fuel data struct and update the fields
            OSMutexPend(&ANGLE_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            AngStr.AoA += angleDelta;
            if(AngStr.AoA > 90){
                AngStr.AoA = 90;
            }
            if(AngStr.AoA < -90){
                AngStr.AoA = -90;
            }

            OSMutexPost(&ANGLE_MUTEX, OS_OPT_POST_ALL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
            updated = 0;
        }
    }
}



static void postgame_task(void * arg){
    RTOS_ERR  err;
    while(1){

    }
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
static void LEDON0_task(void *arg){
    // Drive the leds based on input
    RTOS_ERR  err;
    OS_SEM_CTR  ctr;
    uint16_t freq = 0;
    uint16_t dutyCycle = 0;
    while(1){
        // Pend on LEDON semaphore
        ctr = OSSemPend(&LEDON0_SEM, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        ////////////////////////////////////////////////////////////////////////
        // Pend on LED Duty Cycle Mutex for the data structure
        OSMutexPend(&DUTYCYC0_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        freq = DC0.frequency;
        dutyCycle = DC0.dutyCycle;

        // Release Mutex on Duty Cycle Mutex
        OSMutexPost(&DUTYCYC0_MUTEX, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // Turn on LED
        GPIO_PinOutSet(LED0_port, LED0_pin);
        // Block for amount of time specified by Duty Cycle
        OSTimeDly((freq * dutyCycle), OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        ////////////////////////////////////////////////////////////////////////
        // Post to LEDOFF semaphore
        ctr = OSSemPost(&LEDOFF0_SEM, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));



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
static void LEDOFF0_task(void *arg){
    // Drive the leds based on input
    RTOS_ERR  err;
    OS_SEM_CTR  ctr;
    uint16_t freq = 0;
    uint16_t dutyCycle = 0;
    while(1){
        // Pend on LEDOFF semaphore
        ctr = OSSemPend(&LEDOFF0_SEM, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        ////////////////////////////////////////////////////////////////////////
        // Pend on LED Duty Cycle Mutex for the data structure
        OSMutexPend(&DUTYCYC0_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        freq = DC0.frequency;
        dutyCycle = DC0.dutyCycle;

        // Release Mutex on Duty Cycle Mutex
        OSMutexPost(&DUTYCYC0_MUTEX, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        GPIO_PinOutClear(LED0_port, LED0_pin);

        OSTimeDly((freq *(1 - dutyCycle)), OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        ////////////////////////////////////////////////////////////////////////

        // Post to LEDON semaphore
        ctr = OSSemPost(&LEDON0_SEM, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    }
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
static void LEDON1_task(void *arg){
    // Drive the leds based on input
    RTOS_ERR  err;
    OS_SEM_CTR  ctr;
    uint16_t freq = 0;
    uint16_t dutyCycle = 0;
    while(1){
        // Pend on LEDON semaphore
        ctr = OSSemPend(&LEDON1_SEM, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        ////////////////////////////////////////////////////////////////////////
        // Pend on LED Duty Cycle Mutex for the data structure
        OSMutexPend(&DUTYCYC1_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        freq = DC1.frequency;
        dutyCycle = DC1.dutyCycle;

        // Release Mutex on Duty Cycle Mutex
        OSMutexPost(&DUTYCYC1_MUTEX, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // Turn on LED
        GPIO_PinOutSet(LED1_port, LED1_pin);

        OSTimeDly((freq * dutyCycle), OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        ////////////////////////////////////////////////////////////////////////
        // Post to LEDOFF semaphore
        ctr = OSSemPost(&LEDOFF1_SEM, OS_OPT_POST_ALL, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

        // Block for amount of time specified by Duty Cycle

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
static void LEDOFF1_task(void *arg){
    // Drive the leds based on input
        RTOS_ERR  err;
        OS_SEM_CTR  ctr;
        uint16_t freq = 0;
        uint16_t dutyCycle = 0;
        while(1){
            // Pend on LEDOFF semaphore
            ctr = OSSemPend(&LEDOFF1_SEM, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            ////////////////////////////////////////////////////////////////////////
            // Pend on LED Duty Cycle Mutex for the data structure
            OSMutexPend(&DUTYCYC1_MUTEX, 0, OS_OPT_PEND_BLOCKING, DEF_NULL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            freq = DC1.frequency;
            dutyCycle = DC1.dutyCycle;

            // Release Mutex on Duty Cycle Mutex
            OSMutexPost(&DUTYCYC1_MUTEX, OS_OPT_POST_ALL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

            GPIO_PinOutClear(LED1_port, LED1_pin);

            OSTimeDly((freq *(1 - dutyCycle)), OS_OPT_TIME_DLY, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
            ////////////////////////////////////////////////////////////////////////

            // Post to LEDON semaphore
            ctr = OSSemPost(&LEDON1_SEM, OS_OPT_POST_ALL, &err);
            EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        }
}



//
///***************************************************************************//**
// * @brief
// *      Interrupt Handler for even GPIO interrupts
// *
// * @details
// *      All even interrupts will be cleared from the GPIO Int flag register.
// *      This function will then request the state of pushButton0 and update the
// *      global variable within pushbutton.c. Both negedge and posedge interrupts
// *      are enabled.
// *
// * @note
// *      This interrupt is trigger for PB0, as it is on PortF - pin 6
// *
// *
// ******************************************************************************/
//void GPIO_EVEN_IRQHandler(void){
//    // Clear all even pin interrupt flags
//    GPIO_IntClear(EVEN_GPIO_IRPT);
//    RTOS_ERR err;
//    OS_SEM_CTR  ctr;
//    update_button0State();
//    update_button1State();
//    uint8_t buttonSt = (get_buttonState1() << 1) | get_buttonState0();
//}
//
//
///***************************************************************************//**
// * @brief
// *      Interrupt Handler for odd GPIO interrupts
// *
// * @details
// *      All odd interrupts will be cleared from the GPIO Int flag register.
// *      This function will then request the state of pushButton1 and update the
// *      global variable within pushbutton.c. Only the negedge interrupts
// *      are enabled and valid.
// *
// * @note
// *      This interrupt is trigger for PB1, as it is on PortF - pin 7
// *
// *
// ******************************************************************************/
//void GPIO_ODD_IRQHandler(void){
//    // Clear all even pin interrupt flags
//    GPIO_IntClear(ODD_GPIO_IRPT);
//    RTOS_ERR err;
//    OS_SEM_CTR  ctr;
//    update_button0State();
//    update_button1State();
//    uint8_t buttonSt = (get_buttonState1() << 1) | get_buttonState0();
//}
//


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


