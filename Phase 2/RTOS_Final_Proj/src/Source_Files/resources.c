#include "resources.h"

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
OS_SEM LEDON_SEM;
OS_SEM LEDOFF_SEM;
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
OS_MUTEX DUTYCYC_MUTEX;

void resources_init(){
    RTOS_ERR err;

    //////////////////////////////////////////////////////////
    // Create all Mutexes
    OSMutexCreate(&ANGLE_MUTEX, "Angle Mut", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSMutexCreate(&FUEL_MUTEX, "Fuel Mut", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSMutexCreate(&PHYS_MUTEX, "Physics Mut", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSMutexCreate(&LCD_MUTEX, "LCD Mut", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSMutexCreate(&DUTYCYC_MUTEX, "Duty Cycle Mut", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    //////////////////////////////////////////////////////////
    // Create all flags
    OSFlagCreate(&ANGLE_FLAG, "AoA Flag", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSFlagCreate(&PHYS_FLAG, "Physics Flag", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSFlagCreate(&LCD_FLAG, "LCD Flag", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    //////////////////////////////////////////////////////////
    // Create all Semaphores
    OSSemCreate(&FUEL_SEM, "Fuel Semaphore", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate(&LEDON_SEM, "Led On Semaphore", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate(&LEDOFF_SEM, "Led Off Semaphore", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

    OSSemCreate(&PG_SEM, "PG Semaphore", &err);
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

