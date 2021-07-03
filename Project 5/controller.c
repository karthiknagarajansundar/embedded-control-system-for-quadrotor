#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "controller.h"
#include "constants.h"

void controlSystemTask(void *pvParameters) {
    struct ControlSystemParams *params =
        (struct ControlSystemParams*)pvParameters;

    // we keep local copies of the global state + semaphores
    unsigned short motors[4];
    double gyro_data[3];
    double acc_data[3];
    double r_rpdy[3];
    double estimate[3] = {0.0};

    // copy the semaphore handles for convenience
    SemaphoreHandle_t motors_sem = params->motors_sem;
    SemaphoreHandle_t references_sem = params->references_sem;
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    // LQR gain 
    const double K_lqr[4][5] = {
	{-0.1772,   -0.2627,   -0.0097,   -0.0144,   -0.0583},
	{-0.1772,    0.2627,   -0.0097,    0.0144,    0.0583},
	{0.1772,   0.2627,    0.0097,    0.0144,   -0.0583},
	{0.1772,   -0.2627,    0.0097,   -0.0144,    0.0583} };

    double estimate_d2r[3];
    double gyro_d2r[3];
    double r_rpdy_d2r[3];
    double Errors[5] = {0.0};
    double m = crazyflie_constants.m;
    const double thrust = m/(0.06/65536);;
    const double t = (1.0/crazyflie_constants.pwm_n);

    while(1) {
        // read sensor data (gyro)
	xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
	xSemaphoreGive(sensors_sem);

        // read filter data (angle estimates)
	xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(estimate, params->estimate, sizeof(estimate));
	xSemaphoreGive(estimate_sem);

        // read latest references
	xSemaphoreTake(references_sem, portMAX_DELAY);
        memcpy(r_rpdy, params->r_rpdy, sizeof(r_rpdy));
	xSemaphoreGive(references_sem);

	// Conversion to radians
	for(int i=0; i<3; i++) {
	    gyro_d2r[i] = gyro_data[i] * (M_PI/180) ;
	    estimate_d2r[i] = estimate[i] * (M_PI/180);
	    r_rpdy_d2r[i] = r_rpdy[i] * (M_PI/180);
	}

        // compute error
	Errors[0] = r_rpdy_d2r[0] - estimate_d2r[0];
	Errors[1] = r_rpdy_d2r[1] - estimate_d2r[1]; 
	Errors[2] = 0.0 - gyro_d2r[0];
	Errors[3] = 0.0 - gyro_d2r[1];
	Errors[4] = r_rpdy_d2r[2] - estimate_d2r[2];

	// Computing Motor values
	xSemaphoreTake(motors_sem, portMAX_DELAY);
	double sum = 0.0;
	for (int i = 0; i < 4; i++) {
      	    for (int j = 0; j < 5; j++) {
                sum = sum + K_lqr[i][j]*Errors[j];
 	    }
            motors[i] = (int)t*sum + thrust;	
            sum = 0.0;
        }
	xSemaphoreGive(motors_sem);

	// Logging motor values
	params->log_data[0] = motors[0];
        params->log_data[1] = motors[1];
	params->log_data[2] = motors[2];
        params->log_data[3] = motors[3];

        // write motor output
	xSemaphoreTake(motors_sem, portMAX_DELAY);
        memcpy(params->motors, motors, sizeof(motors));
	xSemaphoreGive(motors_sem);

        // sleep 10ms to make this task run at 100Hz
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
