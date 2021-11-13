#include <iostream>
using namespace std;

#define AOA_TEST        0b1
#define FUEL_TEST       0b10
#define BODC_TEST       0b100
#define GAME_TEST       0b1000
#define HEALTHY_TEST    0b10000
#define CAPS_TEST       0b100000
#define BUTTON_TEST     0b1000000
#define GAMEEND_TEST    0b10000000
#define MAXFUEL_TEST    0b100000000
#define PILOT_TEST      0b1000000000

int AOA_BOUNDS_TEST(){
    return err_code;
}

int FUEL_TANK_RUNSOUTS_TEST(){
    return err_code;
}

int BLACK_OUT_DUTY_CYCLE_TEST(){
    return err_code;
}

int GAMEOVER_DUTY_CYCLE_TEST(){
    return err_code;
}

int HEALTHY_FLIGHT_DUTY_CYCLE_TEST(){
    return err_code;
}

int ALL_FOUR_CAP_SENSED_TEST(){
    return err_code;
}

int BUTTON_INCREMENT_DECREMENT_TEST(){
    return err_code;
}

int GAME_ENDS_AT_LCD_BOUNDARY_TEST(){
    return err_code;
}

int MAX_FUEL_BURNED_TEST(){
    return err_code;
}

int PILOT_IS_BLOCKED_DURING_BLOCKOUT_TEST(){
    return err_code;
}


void runtests(){
    int errCodes = 0;
    errCodes |= AOA_BOUNDS_TEST();
    errCodes |= FUEL_TANK_RUNSOUTS_TEST();
    errCodes |= BLACK_OUT_DUTY_CYCLE_TEST();
    errCodes |= GAMEOVER_DUTY_CYCLE_TEST();
    errCodes |= HEALTHY_FLIGHT_DUTY_CYCLE_TEST();

    errCodes |= ALL_FOUR_CAP_SENSED_TEST();
    errCodes |= BUTTON_INCREMENT_DECREMENT_TEST();
    errCodes |= GAME_ENDS_AT_LCD_BOUNDARY_TEST();
    errCodes |= MAX_FUEL_BURNED_TEST();
    errCodes |= PILOT_IS_BLOCKED_DURING_BLOCKOUT_TEST();

    return;
}
