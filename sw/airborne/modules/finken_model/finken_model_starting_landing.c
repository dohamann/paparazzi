/*
 * Copyright (C) 2014 Andreas Pfohl
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/finken_model/finken_model_starting_landing.h"
#include "subsystems/navigation/common_flight_plan.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_environment.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

// TODO: sane values
#ifndef FINKEN_SYSTEM_P
#define FINKEN_SYSTEM_P 0.075
#endif

#ifndef FINKEN_SYSTEM_I
#define FINKEN_SYSTEM_I 0.00
#endif

#ifndef FINKEN_THRUST_P
#define FINKEN_THRUST_P /* 0.15 */0.10
#endif

#ifndef FINKEN_THRUST_I
#define FINKEN_THRUST_I /*  0.05  */0.05
#endif


#ifndef FINKEN_SYSTEM_UPDATE_FREQ
#define FINKEN_SYSTEM_UPDATE_FREQ 30
#endif

#ifndef FINKEN_VERTICAL_VELOCITY_FACTOR
#define FINKEN_VERTICAL_VELOCITY_FACTOR 0.04
#endif

struct system_model_s finken_starting_landing_model;
//struct actuators_model_s finken_actuators_set_point;

bool finken_has_started = false;
bool finken_has_landed = false;

uint16_t finken_last_stage_time = 0;

void update_actuators_set_point(void);

void finken_starting_landing_model_init(void) {
  finken_starting_landing_model.distance_z     = 0.0;
  finken_starting_landing_model.velocity_theta = 0.0;
  finken_starting_landing_model.velocity_x     = 0.0;
  finken_starting_landing_model.velocity_y     = 0.0;

	finken_actuators_set_point.alpha  = 0.0;
	finken_actuators_set_point.beta   = 0.0;
	finken_actuators_set_point.theta  = 0.0;
	finken_actuators_set_point.thrust = 0.0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_MODEL", send_finken_starting_landing_model_telemetry);
}

void finken_starting_landing_model_periodic(void)
{
	update_finken_starting_landing_model();

    //smooth starting
    
    
    
    //smooth landing
    if( stage_time < 25){
        finken_last_stage_time = stage_time;
    }
    
    
    if( stage_time >= 5 && stage_time < 10 ){
        finken_system_set_point.distance_z = 0.45;
        
    } else if( stage_time >= 10 && stage_time < 15 ){
        finken_system_set_point.distance_z = 0.35;
        
    } else if( stage_time >= 15 && stage_time < 25 ){
        finken_system_set_point.distance_z = 0.25;
        
    } else if( stage_time >= 25 ){

        //decrease thrust manually
        
        while( finken_actuators_set_point.thrust > 0 && stage_time >= (finken_last_stage_time + 1) ){
            
            finken_actuators_set_point.thrust -= 0.05;
            
            if( finken_actuators_set_point.thrust <= 0){
                finken_actuators_set_point.thrust = 0;
                finken_has_landed = true;
                break;
            }
            
            finken_last_stage_time = stage_time;
        }
    }
    
    
}

void update_finken_starting_landing_model(void)
{
	if(finken_sensor_model.distance_z < 2.5) {
		finken_starting_landing_model.distance_z     = finken_sensor_model.distance_z;
	}
	
  finken_starting_landing_model.velocity_theta = finken_sensor_model.velocity_theta;
  finken_starting_landing_model.velocity_x     = finken_sensor_model.velocity_x;
  finken_starting_landing_model.velocity_y     = finken_sensor_model.velocity_y;
}

/*
 * Use finken_starting_landing_set_point to calculate new actuator settings
 */
//float sum_error_x = 0;
//float sum_error_y = 0;
//float sum_error_z = 0;
//float distance_z_old = 0.0;

void send_finken_starting_landing_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_SYSTEM_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_starting_landing_model.distance_z,
    &finken_starting_landing_model.velocity_theta,
    &finken_starting_landing_model.velocity_x,
    &finken_starting_landing_model.velocity_y,
    &finken_actuators_set_point.alpha,
    &finken_actuators_set_point.beta,
    &finken_actuators_set_point.thrust
  );
}

