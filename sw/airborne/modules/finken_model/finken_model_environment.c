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

#include "modules/finken_model/finken_model_environment.h"
#include "subsystems/datalink/telemetry.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"

struct environment_model_s finken_environment_model;

void finken_environment_model_init() {
  finken_environment_model.alpha    = 0.0;
  finken_environment_model.distance = 0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_ENVIRONMENT_MODEL", send_finken_environment_model_telemetry);
}

void finken_environment_model_periodic()
{
  // DO MATH

  finken_environment_model.alpha    = 0.0;
  finken_environment_model.distance = 0;
}

void send_finken_environment_model_telemetry()
{
  DOWNLINK_SEND_FINKEN_ENVIRONMENT_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_environment_model.alpha,
    &finken_environment_model.distance
  );
}
