/*
 * finken_model_pid.h
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */
/**
 * Implementation of a pid-controller with the i-part being the error of the last k steps summed up.
 *
 * This class is based on the lua-implementation
 * of the simulation (see https://github.com/dohamann/paparazzi/blob/master/sim/utilitiesFun.lua)
 */
#ifndef FINKEN_MODEL_PID_H_
#define FINKEN_MODEL_PID_H_

#include <stdint.h>
#define RINGBUFFER_SIZE 6
/**
 * Consists of the pid-'constants',
 * boundaries,
 * a ringbuffer to hold the i-part
 * and some values for telemetry/debugging.
 *
 */
struct pid_controller {
	float p, i, d; ///< pid-constants

	float min; 		///< minimum of output boundaries.
	float max; 		///< maximum of output boundaries.
	int checkMinMax; ///< flag indicating that the output will be truncated to fit [min,max]

	float output; ///< Output value cached for telemetry = pPart + dPart + iPart
	float t; 	 ///< timespan between two iterations cached for telemetry
	float iPart; ///< i-Part of the controller cached for telemetry = sum of ringbuffer * i
	float pPart; ///< p-Part of the controller cached for telemetry = error * p
	float dPart; ///< d-Part of the controller cached for telemetry = (error - previousError) * d

	float previousError; ///< error of the last iteration

	float ringbuffer[RINGBUFFER_SIZE]; ///< Holds the errors of the last k steps.
	int index; ///< Curent position in the ringbuffer i.e. position of the next value to be overwritten.
	int k = RINGBUFFER_SIZE; ///< Size of the ringbuffer
};

extern void setMinMax(float minParam, float maxParam,
		struct pid_controller *con);
/**
 * Calling this method resembles performing one iteration.
 * It updates all values in the struct.
 * @param timeStep time passed since the last iteration.
 * @param error difference between the target value and the current value
 * @param con the controller to adjust
 */
extern float adjust(float error, float timeStep, struct pid_controller *con);
extern void initWallController(struct pid_controller *con);
/**
 *
 */
extern void addIPart(struct pid_controller *con, float iPart);
extern void initFloatController(struct pid_controller *con);

#endif /* FINKEN_MODEL_PID_H_ */
