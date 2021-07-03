#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "filter.h"
#include "constants.h"

void filterTask(void *pvParameters) {
    struct FilterParams *params =
        (struct FilterParams*)pvParameters;

    // we keep local copies of the global state + semaphores
    double gyro_data[3];
    double acc_data[3];

    // copy the semaphore handles for convenience
    SemaphoreHandle_t sensors_sem = params->sensors_sem;
    SemaphoreHandle_t estimate_sem = params->estimate_sem;

    // local internal state.
    double estimate[3] = {0.0};
    double angles_acc[2];
    double gamma = 0.5;
    double h = 1.0/1000;

    while(1) {
        // read sensor data
	xSemaphoreTake(sensors_sem, portMAX_DELAY);
        memcpy(gyro_data, params->gyro_data, sizeof(gyro_data));
        memcpy(acc_data, params->acc_data, sizeof(acc_data));
	xSemaphoreGive(sensors_sem);

        // apply filter
	// Roll and Pitch estimation from accelerometer
	angles_acc[0] = atan2(acc_data[1], acc_data[2]);
	angles_acc[1] = atan2(-acc_data[0], (double) sqrt(pow(acc_data[1],2.0) + pow(acc_data[2],2.0)));

	// Complementary filter
	xSemaphoreTake(estimate_sem, portMAX_DELAY);	
        estimate[0] = gamma*(estimate[0] + h*gyro_data[0]) + ((1-gamma)*angles_acc[0])*(180/M_PI);	
	estimate[1] = gamma*(estimate[1] + h*gyro_data[1]) + ((1-gamma)*angles_acc[1])*(180/M_PI);
	estimate[2] = gyro_data[2];	

	xSemaphoreGive(estimate_sem);	

        // write estimates output
	xSemaphoreTake(estimate_sem, portMAX_DELAY);
        memcpy(params->estimate, estimate, sizeof(estimate));
	xSemaphoreGive(estimate_sem);

        // sleep 1ms to make this task run at 1000Hz
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
