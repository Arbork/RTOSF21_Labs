#ifndef SRC_HEADER_FILES_RESOURCES_H_
#define SRC_HEADER_FILES_RESOURCES_H_

#include "os.h"
#include "em_emu.h"

//////////////////////////////
///
/// Creating OS flags for tasks
///
/// /////////////////////////
OS_FLAG_GRP ANGLE_FLAG;
OS_FLAG_GRP PHYS_FLAG;
OS_FLAG_GRP LCD_FLAG;

//////////////////////////////
///
/// Creating OS Semaphores for tasks
///
/// /////////////////////////
OS_SEM FUEL_SEM;
OS_SEM LEDON0_SEM;
OS_SEM LEDOFF0_SEM;
OS_SEM LEDON1_SEM;
OS_SEM LEDOFF1_SEM;
OS_SEM PG_SEM;

//////////////////////////////
///
/// Creating OS Mutexes for Data
///
/// /////////////////////////
OS_MUTEX ANGLE_MUTEX;
OS_MUTEX FUEL_MUTEX;
OS_MUTEX PHYS_MUTEX;
OS_MUTEX LCD_MUTEX;
OS_MUTEX DUTYCYC0_MUTEX;
OS_MUTEX DUTYCYC1_MUTEX;



void resources_init(void);


#endif /* SRC_HEADER_FILES_RESOURCES_H_ */
