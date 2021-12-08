#ifndef SRC_HEADER_FILES_TASK_H_
#define SRC_HEADER_FILES_TASK_H_

#include "gpio.h"
#include "capsense.h"
#include "pushbutton.h"
#include "em_emu.h"
#include "queue.h"
//#include "resources.h"
#include "os.h"
#include "sl_board_control.h"
#include "sl_simple_button_instances.h"
#include "glib.h"
#include "dmd.h"
#include "math.h"
//#include "math.c"
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
#define PHYS_TASK_STACK_SIZE      1024
#define PHYS_TASK_PRIO            5

// Landing
#define LAND_TASK_STACK_SIZE      1024
#define LAND_TASK_PRIO            6

// LCD
#define LCD_TASK_STACK_SIZE      1024
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
    uint32_t fuelLeft;      // in [kg]
    uint32_t thrustPrct;    // Percent of max thrust at any given time
    uint32_t consRate;      // Units of fuel consumed per second
    uint32_t maxThrust;
};
#define MAXTHRUST       5   // Fuel units per second
#define THRUST_QUANTA   10  // Percent of thrust incremented or decremented per Fuel freq

// Configuration Data Structure
struct configStr{
    uint32_t fuelMass;
    uint8_t STR_VERSION;    // Version of this data structure
    uint32_t gravity;       // Acceleration due to gravity
    uint32_t vehMass;       // Mass of Vehicle
    int32_t xMin;           // Graphing Limits
    int32_t xMax;           // Graphing Limits
    uint32_t maxThrust;     // In Newtons
    uint32_t conversionEff; // Conversion efficiency (fuel->thrust)
    uint32_t fuelNrgDens;   // Density of energy per gram
    uint32_t maxLandSpdH;   // Maximum speed player can successfully land at Horizontally [mm/s]
    uint32_t maxLandSpdV;   // Maximum speed player can successfully land at Vertically   [cm/s]
    int32_t initVeloV;      // [cm/s]
    int32_t initVeloH;      // [cm/s]
    uint32_t blkOutAccel;   // Threshold of acceleration for blacking out   [mm/s^2]
    uint32_t blkOutDura;    // Duration of blackout         [s]
    int32_t initX;          // Initial Horizontal location  [mm]
};

#define VERSION     1
#define GRAVITY     9.81    // [mm/s^2]
#define VEHMASS     2000
#define FUEL_MASS   2000
#define X_MIN       0   // Left edge
#define X_MAX       128 // Right edge
#define X_INIT      64  // Roughly in the middle of the screen
#define MAX_THRUST   0// define      // [mm/s^2]
#define NRG_DENS    0//define       // [J/kg]
#define INIT_VELOX  5               // [cm/s]
#define INIT_VELOY  -2              // [cm/s]
#define BO_ACCEL    (5 * GRAVITY)   // [mm/s^2]
#define BO_DURA     5000   // [ms]
#define MAX_LANDX   5   // [mm/s]
#define MAX_LANDY   20   // [mm/s]
// AoA Struct
struct AngleStr{
    int8_t AoA;
};
#define LEFT_MOST_PAD       -4  // Change of angle
#define LEFT_INNER_PAD      -2  // Change of angle
#define RIGHT_INNER_PAD      2  // Change of angle
#define RIGHT_MOST_PAD       4  // Change of angle
#define LEFT_BOUND          -90 // Left bound of angle
#define RIGHT_BOUND          90 // Right bound of angle
#define LAND_LA             -10
#define LAND_RA              10

///////////////////////////////////////////////////////////
// Physics Struct
struct PhysStr{
    int32_t     xVelo;
    int32_t     yVelo;
    int32_t     xPos;
    int32_t     yPos;
    int32_t     xAccel;
    int32_t     yAccel;
};

enum bitSpec{
    buttonBoth,  //!< buttonBoth
    button2,     //!< button2
    button1,     //!< button1
    buttonNeither//!< buttonNeither
};

// LCD bit specs
#define UPDATE_LCD  0b1
#define SHIP_BO     0b10
#define SHIP_LAND   0b100
#define SHIP_CRASH  0b1000


#define LED_FLAG_ALL 0xf
#define APP_FLAG_ALL 0x3
#define LCD_FLAG_ALL 0xf

#define BO_STR  "Blacked Out"
///////////////////////////////////////////////////////////////////////////////
// Update frequencies
#define PHYS_FREQ   100 // [ms]
#define LAND_CNT    (BO_DURA / PHYS_FREQ)
#define DISP_FREQ   100 // [ms]


//////////////////////////////////////////////////////////////////////////////
// LCD ship stuff

#define SHIP_TOP        "    *  "
#define SHIP_MIDDLE1    "   *** "
#define SHIP_MIDDLE2    "  *****"
#define SHIP_MIDDLE3    " *******"
#define SHIP_BOTTOM     "*********"

void task_init(void);
void queue_init(void);
void GPIO_EVEN_IRQHandler(void);
void GPIO_ODD_IRQHandler(void);
void rocket_init(void);
void resources_init(void);


#endif /* SRC_HEADER_FILES_TASK_H_ */
