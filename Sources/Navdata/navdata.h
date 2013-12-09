#ifndef _NAVDATA_H_
#define _NAVDATA_H_

#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Video/video_stage.h>
#include <inttypes.h>
#include <gtk/gtk.h>

typedef struct navdata_data1_t //define the structure of control data;
{
    float phi;//Left/right angle between -1 to +1 - negative values bend leftward.
    float psi;//Front/back angle between -1 to +1 - negative values bend forward.
    float theta;//gaz Vertical speed - negative values make the drone go down.
    float altitude;//yaw Angular speed - negative values make the drone spin left.
    int start;
    float vx;
    float vy;
    float vz;

}navdata_data1_t;

navdata_data1_t navigation_data;

static vp_os_mutex_t  navigation_data_lock = PTHREAD_MUTEX_INITIALIZER;

#endif // _NAVDATA_H_
