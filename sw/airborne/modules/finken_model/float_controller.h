/**
 * The idea behind the float_controller was to reduce drifting 
 * and the residual velocity when dodging from a wall.
 * Sadly it was too unstable because of the sensor quality.
 */
#ifndef SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_
#define SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_

#include "finken_model_pid.h"
#include "modules/finken_model/finken_model_system.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

extern struct pid_controller xFinkenFloatController;
extern struct pid_controller yFinkenFloatController;

extern void float_controller_init(void);
extern void float_controller_periodic(void);
/**
 * @return The difference in distance between the current and the last timestep on the x-axis.
 * Uses the sensor with the lowest current distance, as the sensors error is smaller for short distances.
 */
extern int getXDistanceDiff(void);
/**
 * @return The difference in distance between the current and the last timestep on the y-axis.
 * Uses the sensor with the lowest current distance, as the sensors error is smaller for short distances.
 */
extern int getYDistanceDiff(void);

#endif /* SW_AIRBORNE_MODULES_FINKEN_MODEL_FLOAT_CONTROLLER_H_ */
