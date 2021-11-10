#ifndef SRC_HEADER_FILES_TASK_H_
#define SRC_HEADER_FILES_TASK_H_

#include "gpio.h"
#include "capsense.h"
#include "pushbutton.h"
#include "em_emu.h"
#include "queue.h"
#include "resources.h"
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
// Physics Engine
#define PHYS_TASK_STACK_SIZE      128
#define PHYS_TASK_PRIO            5


// Landing
#define LAND_TASK_STACK_SIZE      512
#define LAND_TASK_PRIO            9


// LCD
#define LCD_TASK_STACK_SIZE      512
#define LCD_TASK_PRIO            10


// Fuel
#define FUEL_TASK_STACK_SIZE    512
#define FUEL_TASK_PRIO          11
#define FUEL_POLL_FREQ          500 // Polls every 500ms



// Angle of Attack
#define AOA_TASK_STACK_SIZE     512
#define AOA_TASK_PRIO           11
#define AOA_POLL_FREQ           500

// Post Game
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

static volatile uint8_t CAP_STATE;       // Private variable holding the state of the capsensor


#define BTN_mask        0x00000003
#define CAP_mask        0x0000000c


///////////////////////////////////////////////////////////////////////////////
// Data Structures

struct DutyCycle{
    uint16_t frequency;
    uint16_t dutyCycle;
};

#define BLACKOUT_FREQ   3000 //mHz
#define GAMEOVER_FREQ   1000
#define MAXTHRUST_FREQ  3000
#define NOTHEALTHY_DC   0.5

// Fuel Struct
struct FuelStr{
    uint32_t fuelLeft;
    uint32_t thrustPrct;    // Percent of max thrust at any given time
    uint32_t consRate;      // Units of fuel consumed per second
};
#define MAXTHRUST       5   // Fuel units per second
#define THRUST_QUANTA   10  // Percent of thrust incremented or decremented per Fuel freq


// AoA Struct
struct AngleStr{
    uint8_t AoA;
};
#define LEFT_MOST_PAD       -4
#define LEFT_INNER_PAD      -2
#define RIGHT_INNER_PAD      2
#define RIGHT_MOST_PAD       4

enum bitSpec{
    buttonNone,
    button0,
    button1,
    buttonBoth
};


#define LED_FLAG_ALL 0xf
#define APP_FLAG_ALL 0x3






void task_init(void);
void queue_init(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);

#endif /* SRC_HEADER_FILES_TASK_H_ */
