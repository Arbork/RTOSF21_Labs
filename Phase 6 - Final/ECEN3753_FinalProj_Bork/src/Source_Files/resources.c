//#include "resources.h"
//
//
//void resources_init(void){
//    RTOS_ERR err;
//
//    //////////////////////////////////////////////////////////
//    // Create all Mutexes
//    OSMutexCreate(&ANGLE_MUTEX, "Angle Mut", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSMutexCreate(&FUEL_MUTEX, "Fuel Mut", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSMutexCreate(&PHYS_MUTEX, "Physics Mut", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSMutexCreate(&LCD_MUTEX, "LCD Mut", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSMutexCreate(&DUTYCYC0_MUTEX, "Duty Cycle Mut0", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSMutexCreate(&DUTYCYC1_MUTEX, "Duty Cycle Mut1", &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    //////////////////////////////////////////////////////////
//    // Create all flags
//    OSFlagCreate(&ANGLE_FLAG, "AoA Flag", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSFlagCreate(&PHYS_FLAG, "Physics Flag", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSFlagCreate(&LCD_FLAG, "LCD Flag", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    //////////////////////////////////////////////////////////
//    // Create all Semaphores
//    OSSemCreate(&FUEL_SEM, "Fuel Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSSemCreate(&LEDON0_SEM, "Led On 0 Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSSemCreate(&LEDOFF0_SEM, "Led Off 0 Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSSemCreate(&LEDON1_SEM, "Led On 1 Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSSemCreate(&LEDOFF1_SEM, "Led Off 1 Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//    OSSemCreate(&PG_SEM, "PG Semaphore", 0, &err);
//    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
//
//}
//
