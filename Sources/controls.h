#ifndef _FK_CONTROLS
#define _FK_CONTROLS

#include <stdio.h>

#define CONTROL_DELAY 30

//int please_take_off;
extern int exit_program;
//extern int control_var;

/**
 * @fn ardrone_at_set_progress_cmd
 * @brief Sends the drone progressive commands
 * @param flag Use 1 << value of ARDRONE_PROGRESSIVE_CMD_FLAG_XXX to use a flag
 * @param phi Left/right angle between -1 to +1 - negative values bend leftward.
 * @param roll Front/back angle between -1 to +1 - negative values bend forward.
 * @param gaz Vertical speed - negative values make the drone go down.
 * @param yaw Angular speed - negative values make the drone spin left.
 * This function allows the client program to control the drone by giving it a front/back
 * and left/right bending order, a vertical speed order, and a rotation order.
 * All values are given as a percentage of the maximum bending angles (in degrees),
 * vertical speed (in millimeters per second) and angular speed (in degrees per second).
 */

typedef struct control_data_t //define the structure of control data;
{
    float phi;//Left/right angle between -1 to +1 - negative values bend leftward.
    float roll;//Front/back angle between -1 to +1 - negative values bend forward.
    float gaz;//gaz Vertical speed - negative values make the drone go down.
    float yaw;//yaw Angular speed - negative values make the drone spin left.
    int start;

}control_data_t;

#endif
