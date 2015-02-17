/*
 * Copyright (C) 2014  Sebastian Mai
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

#include "modules/sonar/sonar_array_i2c.h"
#include "generated/airframe.h"
#include "mcu_periph/i2c.h"
#include "state.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"

/** Sonar offset.
 *  Offset value in m (float)
 *  distance mesured by the i2c sensor
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0.
#endif

/** Sonar scale.
 *  Scaling factor to compute real distances(float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 1.0000
#endif

#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c2
#endif

/** SONAR_ADDR_FRONT
 * 	adress for the front Sensor
 * 	same as RIGHT, LEFT, BACK ... 
 */
#ifndef SONAR_ADDR_FRONT
#define SONAR_ADDR_FRONT 0x71
#endif
#ifndef SONAR_ADDR_RIGHT
#define SONAR_ADDR_RIGHT 0x72
#endif
#ifndef SONAR_ADDR_BACK
#define SONAR_ADDR_BACK 0x73
#endif
#ifndef SONAR_ADDR_LEFT
#define SONAR_ADDR_LEFT 0x74
#endif
#ifndef SONAR_ADDR_DOWN
#define SONAR_ADDR_DOWN 0x75
#endif

static const char read_order[] =  {SONAR_ADDR_DOWN, SONAR_ADDR_FRONT, SONAR_ADDR_RIGHT, SONAR_ADDR_DOWN, SONAR_ADDR_BACK, SONAR_ADDR_LEFT};

float sonar_distance;
uint8_t sonar_status;
uint8_t sonar_index;
struct sonar_values_s sonar_values;
struct sonar_data_available_s sonar_data_available;
#define SONAR_STATUS_IDLE 0
#define SONAR_STATUS_PENDING 1

struct i2c_transaction sonar_i2c_read_trans;
struct i2c_transaction sonar_i2c_write_trans;

void sonar_array_i2c_init(void) {
	sonar_index = 0;
	sonar_status = SONAR_STATUS_IDLE;


	sonar_data_available.front = FALSE;
	sonar_data_available.right = FALSE;
	sonar_data_available.back  = FALSE;
	sonar_data_available.left  = FALSE;
	sonar_data_available.down  = FALSE;

	sonar_values.front = 0;
	sonar_values.right = 0;
	sonar_values.back  = 0;
	sonar_values.left  = 0;
	sonar_values.down  = 0;

	sonar_i2c_read_trans.status = I2CTransDone;
	sonar_i2c_write_trans.status = I2CTransDone;
}


/** sonar_send_command
 *	send take_range_reading command (0x51) to the sonar sensors to trigger the range readin
 */
void sonar_send_command(uint8_t i2c_addr) {
	if (sonar_i2c_write_trans.status == I2CTransDone) {
		sonar_i2c_write_trans.buf[0] = 0x51;
		i2c_transmit(&SONAR_I2C_DEV, &sonar_i2c_write_trans, i2c_addr << 1, 1); // 7-Bit Adress + write Bit
	}
	sonar_i2c_write_trans.status = I2CTransDone;
}

/** Read I2C value to update sonar measurement and request new value
 */
void sonar_array_i2c_periodic(void) {
#ifndef SITL
	sonar_index++;
	sonar_index %= 6;
	sonar_send_command(read_order[sonar_index]);
	sonar_status = SONAR_STATUS_PENDING;

#else // SITL
#warn "SITL not implemented for sonar_array_i2c yet"
#endif // SITL
}

void sonar_array_i2c_event( void ) {
#ifndef SITL
	if (sonar_status == SONAR_STATUS_PENDING && sonar_i2c_read_trans.status == I2CTransDone) {
    if(i2c_receive(&SONAR_I2C_DEV, &sonar_i2c_read_trans, (read_order[sonar_index] << 1) | 1, 2)) {
			sonar_status = SONAR_STATUS_IDLE;
			uint16_t meas = ((uint16_t)(sonar_i2c_read_trans.buf[0]) << 8) | (uint16_t)(sonar_i2c_read_trans.buf[1]);	// recieve mesuarment

			if(meas > 0) {
				switch(sonar_index) {
					case 1:
						sonar_values.front = meas;
						sonar_data_available.front = TRUE;
						break;
					case 2:
						sonar_values.right = meas;
						sonar_data_available.right = TRUE;
						break;
					case 4:
						sonar_values.back = meas;
						sonar_data_available.back = TRUE;
						break;
					case 5:
						sonar_values.left = meas;
						sonar_data_available.left = TRUE;
						break;
					default:
						sonar_values.down = meas;
						sonar_data_available.down = TRUE;
				}
				sonar_status = SONAR_STATUS_IDLE;
			}

		}

	}
  sonar_i2c_read_trans.status = I2CTransDone;
#endif
}
