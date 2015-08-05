#include "wall_avoidance_controller.h"

struct pid_controller frontPIDController;
struct pid_controller rightPIDController;
struct pid_controller backPIDController;
struct pid_controller leftPIDController;

float TOLERABLE_PROXY_DIST = 80; ///< minimum treshold - if less, begin collision avoidance
float MIN_HEIGHT = 0.15; ///< the minimum altitude before we can begin using sonars

void wall_avoidance_controller_init() {
	initWallController(&frontPIDController);
	initWallController(&rightPIDController);
	initWallController(&backPIDController);
	initWallController(&leftPIDController);
}

void wall_avoidance_controller_periodic() {
	if (finken_system_model.reset) {
		resetAll();
		finken_system_model.reset = 0;
	}

	if (finken_system_model.distance_z > MIN_HEIGHT) {
		int angleCap = 25;
		float front = pid_planar(finken_sensor_model.distance_d_front,
				&frontPIDController);
		float back = pid_planar(finken_sensor_model.distance_d_back,
				&backPIDController);
		float xDegree = ((front - back) / 250) * angleCap;
		alphaComponents[1] = xDegree;

		float left = pid_planar(finken_sensor_model.distance_d_left,
				&leftPIDController);
		float right = pid_planar(finken_sensor_model.distance_d_right,
				&rightPIDController);
		float yDegree = ((left - right) / 250) * angleCap;
		betaComponents[1] = yDegree;
	}
	updateActuators();
}

void resetAll(bool reset) {
	resetIPart(&frontPIDController);
	resetIPart(&backPIDController);
	resetIPart(&leftPIDController);
	resetIPart(&rightPIDController);
	resetIPart(&xFinkenFloatController);
	resetIPart(&yFinkenFloatController);
}

float max_(float x, float y) {
	return (x < y) ? y : x;
}

float pid_planar(float sonar_dist, struct pid_controller *pid) {
	float error = max_(TOLERABLE_PROXY_DIST - sonar_dist, 0.0);
	return adjust(error, 0.03, pid);
}
