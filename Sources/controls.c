#include <controls.h>
#include <Video/display_stage.h>

//ARDroneLib
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Control/ardrone_control.h>

//Common
#include <config.h>
#include <ardrone_api.h>

//VP_SDK
#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Os/vp_os_signal.h>

//Local project

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <math.h>

#include <ardrone_tool/Control/ardrone_control_configuration.h>
#include <ardrone_tool/ardrone_tool_configuration.h>

#include <Navdata/navdata.h>

/**
 * @fn ardrone_at_set_progress_cmd
 * @brief Sends the drone progressive commands
 * @param flag Use 1 << value of ARDRONE_PROGRESSIVE_CMD_FLAG_XXX to use a flag
 * @param phi Left/right angle between -1 to +1 - negative values bend leftward.
 * @param roll Front/back angle between -1 to +1 - negative values bend forward.
 * @param gaz Vertical speed - negative values make the drone go down.
 * @param yaw Angular speed - negative values make the drone spin left.
 * @param magneto_psi floating value between -1 to +1.
 * @param magneto_psi_accuracy floating value between -1 to +1
 * This function allows the client program to control the drone by giving it a front/back
 * and left/right bending order, a vertical speed order, and a rotation order.
 * All values are given as a percentage of the maximum bending angles (in degrees),
 * vertical speed (in millimeters per second) and angular speed (in degrees per second).
 */

int direction;

control_data_t update_control_data(CvPoint gravity_center_c,int keyboard)
{
    control_data_t control_data1={.start=-1, .phi=0, .yaw=0, .roll=0, .gaz=0};
    static int prev_keyboard;
    static int prev_start;
    static float phi_prev, roll_prev;
    if(keyboard!=prev_keyboard)
    {
        switch (keyboard&255)
        {
            case 32:
            {
                if(prev_start)
                {
                    control_data1.start=0;
                    prev_start=0;
                }
                else
                {
                    control_data1.start=1;
                    prev_start=1;
                }
                break;
            }
            case 27:
            {
                exit_program=0;
                control_data1.start=0;
                break;
            }
            case 38:
            {
                direction=(direction+1)%4;
            }
        }
    }
    control_data1.roll=-0.1;//(float)(-0.1);
    control_data1.phi=0;//-0.1;
    //control_data1.phi=(float)(gravity_center.x-320)/640;;
    prev_keyboard=keyboard;

    return control_data1;
}

send_control_data(control_data_t control_data)
{
    static int seq_no;
    if(control_data.start>=0)
    {
        if(control_data.start==0)
        {
            ardrone_tool_set_ui_pad_start(0);
            vp_os_delay(50);
        }
        else
        {
            ardrone_at_set_progress_cmd(0, 0, 0, 0, 0 );
            ardrone_tool_set_ui_pad_start(1);
            ardrone_at_set_progress_cmd(0, 0, 0, 0, 0 );
            vp_os_delay(4000);
        }
    }
    else
    {

        /*switch(direction)
        {
            case 0:
            {
                ardrone_at_set_progress_cmd(1, -1,0,0,0);
                break;
            }
            case 1:
            {
                ardrone_at_set_progress_cmd(1, 0,-0.5,0,0);
                break;
            }
            case 2:
            {
                ardrone_at_set_progress_cmd(1, 0,0,-0.5,0);
                break;
            }
            case 3:
            {
                ardrone_at_set_progress_cmd(1, 0,0,0,-0.5);
                break;
            }
         }*/
        ardrone_at_reset_com_watchdog();
        ardrone_at_set_progress_cmd(1, 0,(float32_t)-1,0,0);
    }


}

static vp_os_mutex_t  control_data_lock = PTHREAD_MUTEX_INITIALIZER;

control_data_t control_data={.start=-1, .phi=0, .yaw=0, .roll=0, .gaz=0};

DEFINE_THREAD_ROUTINE(control_prepare, data)
{
    PRINT( "Initilizing Thread control_prepare\n" );
    CvPoint gravity_center_c=cvPoint(-1,-1);
    int control_var_l=0;
    FILE * fo;
    fo = fopen("telemetry","wt");
    int seq=0;
    double start_time = cvGetTickCount();
    double curr_time = 0;
    double freq = cvGetTickFrequency();
    fprintf(fo, "freq=%f\tstart_time=%f",freq,start_time);
    while(exit_program)
    {
        vp_os_mutex_lock(&gravity_center_lock);
        gravity_center_c=gravity_center;
        vp_os_mutex_unlock(&gravity_center_lock);
        int tmp;
//        if(gravity_center_c.x>=0)
        vp_os_mutex_lock(&key_update_lock);
        control_var_l=control_var;
        vp_os_mutex_unlock(&key_update_lock);

        vp_os_mutex_lock(&control_data_lock);
        control_data=update_control_data(gravity_center_c, control_var_l);
        vp_os_mutex_unlock(&control_data_lock);

        vp_os_mutex_lock(&navigation_data_lock);
        navdata_data1_t ndata=navigation_data;
        vp_os_mutex_unlock(&navigation_data_lock);
        curr_time=cvGetTickCount();
        vp_os_delay(20);
    }
    fclose(fo);
    PRINT( "control_prepare Thread Ending\n" );
    return C_OK;
}


DEFINE_THREAD_ROUTINE( comm_control, data )
{

    PRINT( "\n\n\n*****************Initilizing Thread comm_control************\n\n\n\n" );

    control_data_t control_send;
    while(exit_program)
    {
        vp_os_mutex_lock(&control_data_lock);
        control_send=control_data;
        vp_os_mutex_unlock(&control_data_lock);

        send_control_data(control_send);
    }
	ardrone_tool_set_ui_pad_start(0);
	PRINT( "comm_control Thread Ending\n" );
    return C_OK;
}
