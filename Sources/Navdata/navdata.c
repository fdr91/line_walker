#include <ardrone_tool/Navdata/ardrone_navdata_client.h>

#include <Navdata/navdata.h>
#include <stdio.h>
#ifndef M_PI
#define M_PI  (3.14159265358979323846264338327)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180/M_PI)
#endif
#ifndef KDEG_TO_RAD
#define KDEG_TO_RAD (M_PI/(180*1000))
#endif


/* Initialization local variables before event loop  */
inline C_RESULT demo_navdata_client_init( void* data )
{
  return C_OK;
}

/* Receving navdata during the event loop */
inline C_RESULT demo_navdata_client_process( const navdata_unpacked_t* const navdata )
{
	const navdata_demo_t*nd = &navdata->navdata_demo;
	vp_os_mutex_lock(&navigation_data_lock);
	navigation_data.theta=nd->theta*KDEG_TO_RAD;
	navigation_data.phi=nd->phi*KDEG_TO_RAD;
	navigation_data.psi=nd->psi*KDEG_TO_RAD;
	navigation_data.altitude=nd->altitude*0.001;
	navigation_data.vx=nd->vx;
	navigation_data.vy=nd->vy;
	navigation_data.vz=nd->vz;
	vp_os_mutex_unlock(&navigation_data_lock);
	printf("phi=%3.3f, psi=%3.3f, theta=%3.3f, altitude=%3.3f\n", navigation_data.phi, navigation_data.psi, navigation_data.theta, navigation_data.altitude);
	printf("\033[1A");

	vp_os_delay(1);

  return C_OK;
}

/* Relinquish the local resources after the event loop exit */
inline C_RESULT demo_navdata_client_release( void )
{
  return C_OK;
}

/* Registering to navdata client */
BEGIN_NAVDATA_HANDLER_TABLE
  NAVDATA_HANDLER_TABLE_ENTRY(demo_navdata_client_init, demo_navdata_client_process, demo_navdata_client_release, NULL)
END_NAVDATA_HANDLER_TABLE

