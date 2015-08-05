/**
 * Module for wall-avoidance behavior.
 * Uses one finken_model_pid for each sonar.
 * It uses the filtered distance values from finken_sensor_model
 * and pushes it's output to the finken_actuators_model
 */
#ifndef WALLAVOIDANCECONTROLLER_H_
#define WALLAVOIDANCECONTROLLER_H_

#include "finken_model_pid.h"
#include "modules/finken_model/finken_model_system.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

extern struct pid_controller frontPIDController;
extern struct pid_controller rightPIDController;
extern struct pid_controller backPIDController;
extern struct pid_controller leftPIDController;

/**
 * Sets parameters of pid-controllers.
 */
extern void wall_avoidance_controller_init(void);

/**
 * Calls all pid-controllers, merges the results and pushes it to finken_actuators_model.
 * After changing the flight mode it resets the i-part of all PID-controllers to avoid wind-up.
 */
extern void wall_avoidance_controller_periodic(void);

/**
 * Calls the pid_controller.adjust function. Control x and y movements, try to avoid walls.
 */
extern float pid_planar(float sonar_dist, struct pid_controller *pid);

#endif /* WALLAVOIDANCECONTROLLER_H_ */
