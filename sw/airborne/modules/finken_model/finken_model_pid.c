/*
 * finken_model_pid.c
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#include "finken_model_pid.h"

void setMinMax(float minParam, float maxParam, struct pid_controller *con) {
	con->min = minParam;
	con->max = maxParam;
	con->checkMinMax = 1;
}

float adjust(float error, float timeStep, struct pid_controller *con) {
	con->t = timeStep;
	float derivative = (error - con->previousError) / timeStep;

	if (con->previousError == 0) {
		derivative = 0;
	}
	addIPart(con, error * timeStep);
	con->previousError = error;
	con->pPart = con->p * error;
	con->dPart = con->d * derivative;
	float res = con->pPart + con->iPart + con->dPart;

	if (con->checkMinMax == 1) {
		if (res < con->min) {
			res = con->min;
		} else if (res > con->max) {
			res = con->max;
		}
	}
	//res = -res;
	con->output = res;
	return res;
}

void initWallController(struct pid_controller *con) {
	con->k = RINGBUFFER_SIZE;
	con->p = 2.5;
	con->i = 0;
	con->d = 0.4;
	float cap = 250;
	con->min = -cap;
	con->max = cap;
	con->checkMinMax = 1;
	con->index = 0;
	con->iPart = 0;
}

void initFloatController(struct pid_controller *con) {
	initWallController(con);
	con->p = 0.1;
	con->i = 0.1;
	con->d = 0.1;
	con->checkMinMax = 1;
	float cap = 0;
	con->min = -cap;
	con->max = cap;
}

extern void addIPart(struct pid_controller *con, float i_error) {
	con->index = con->index % con->k;
	con->ringbuffer[con->index] = i_error;
	float sum = 0;
	for (int i = 0; i < con->k; i++) {
		sum += con->ringbuffer[i];
	}
	con->iPart = sum * con->i;
	con->index++;
}

void resetIPart(struct pid_controller *con) {
	for (int i = 0; i < con->k; i++) {
		con->ringbuffer[i] = 0;
	}
}

