#ifndef SRC_HEADER_FILES_TASK_H_
#define SRC_HEADER_FILES_TASK_H_

#include "gpio.h"
#include "capsense.h"
#include "pushbutton.h"
#include "em_emu.h"
#include "queue.h"
#include "os.h"
//#include "os_core.c"
///////////////////////////////////////////////////////////////////////////////
///
/// IDLE task defines
///
/// ///////////////////////////////////////////////////////////////////////////

//
#define IDLE_TASK_STACK_SIZE      96
#define IDLE_TASK_PRIO            32

///////////////////////////////////////////////////////////////////////////////
///
///  Task defines
///
/// ///////////////////////////////////////////////////////////////////////////

#define PHYS_TASK_STACK_SIZE      128
#define PHYS_TASK_PRIO            5

#define LAND_TASK_STACK_SIZE      512
#define LAND_TASK_PRIO            9

#define LCD_TASK_STACK_SIZE      512
#define LCD_TASK_PRIO            10

#define FUEL_TASK_STACK_SIZE      512
#define FUEL_TASK_PRIO            11

#define AOA_TASK_STACK_SIZE      512
#define AOA_TASK_PRIO            11

#define PG_TASK_STACK_SIZE      512
#define PG_TASK_PRIO            12

///////////////////////////////////////////////////////////////////////////////
///
/// LED task defines
///
/// ///////////////////////////////////////////////////////////////////////////

#define LEDOFF_TASK_STACK_SIZE      96
#define LEDOFF_TASK_PRIO            10

#define LEDON_TASK_STACK_SIZE      96
#define LEDON_TASK_PRIO            10

// LED 0 pin is
#define LED0_port   gpioPortF
#define LED0_pin    4u
#define LED0_default  false   // Default false (0) = off, true (1) = on
// LED 1 pin is
#define LED1_port   gpioPortF
#define LED1_pin    5u
#define LED1_default  false // Default false (0) = off, true (1) = on

///////////////////////////////////////////////////////////////////////////////
///
/// Speed Setpoint task defines
///
/// ///////////////////////////////////////////////////////////////////////////

#define SETPOINT_TASK_STACK_SIZE      512
#define SETPOINT_TASK_PRIO            12
#define speedInc    0b10
#define speedDec    0b1

///////////////////////////////////////////////////////////////////////////////
///
/// Vehicle Direction task defines
///
/// ///////////////////////////////////////////////////////////////////////////

#define CHANNEL0 0
#define CHANNEL1 1
#define CHANNEL2 2
#define CHANNEL3 3

#define VDIREC_TASK_STACK_SIZE      512
#define VDIREC_TASK_PRIO            13
static volatile uint8_t CAP_STATE;       // Private variable holding the state of the capsensor


#define BTN_mask        0x00000003

#define CAP_mask        0x0000000c

///////////////////////////////////////////////////////////////////////////////
///
/// Vehicle Monitor task defines
///
/// ///////////////////////////////////////////////////////////////////////////

#define VMON_TASK_STACK_SIZE      512
#define VMON_TASK_PRIO            14
//static volatile uint8_t CAP_STATE;       // Private variable holding the state of the capsensor



///////////////////////////////////////////////////////////////////////////////
// Data Structures

struct SSData{
    uint8_t currSpd;
    uint8_t spdInc;
    uint8_t spdDec;
};

struct VDData{
    uint8_t currDirec;
    uint8_t rightCnt;
    uint8_t leftCnt;
    uint8_t sameDirecCnt;
};

enum bitSpec{
    buttonNone,
    button0,
    button1,
    buttonBoth
};


#define LED_FLAG_ALL 0xf
#define APP_FLAG_ALL 0x3

///////////////////////////
/// app flags
/// ////////////////////////
#define speedFlag    0b1
#define directionFlag   0b10

#define straight    0
#define hardRight   0b1
#define softRight   0b10
#define softLeft    0b100
#define hardLeft    0b1000

///////////////////////////
/// LED flags
/// ////////////////////////
#define NO_VIOS     0b1
#define SPEED_VIO   0b10
#define COLL_ALERT  0b100
#define BOTH_VIOS   0b1000




void task_init(void);
void queue_init(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

#endif /* SRC_HEADER_FILES_TASK_H_ */
