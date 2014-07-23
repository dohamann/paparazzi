/*
 *
 * Copyright (C) 2014, Sebastian Mai, ovgu
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

/**
 * @file modules/nav/nav_finken.h
 * @brief nav routines for finken robots
 */

#ifndef NAV_FINKEN_H
#define NAV_FINKEN_H

#include "std.h"
#include "paparazzi.h"

#include "modules/sonar/sonar_array_i2c.h"

extern float sonar_failsave_pitch( void );
extern float sonar_failsave_roll( void );

#endif
