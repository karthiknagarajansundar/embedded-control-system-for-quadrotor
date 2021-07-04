# Embedded control system design for a quadrotor
### Model-based development of cyber-physical systems
## Abstract
The main objective of this project is modelling and control of the quadcopter (Crazyflie). Various processes like estimation of orientation using complementary filter, Equation-based Modelling (i.e model-based approach), Linearization of the plant, LQR based controller design and evaluation, and respective simulations. The tools used for modelling, evaluation and simulation includes Matlab, Simulink and Simscape. The LQR has been fine tuned for better results of the controller. Finally, all the mentioned tasks were implemented in RTOS with task scheduling using real time C programming.

## Orientation estimation using complementary filter
### Complementary filter model
<img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/compFilt.JPG" width="750" height="450">

### Plots of accelerometer and gyroscope data along the three axes, up-to 90 seconds
<img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/acc_data.jpg" width="400" height="200"> <img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/gyro_data.jpg" width="400" height="200">

### Pitch and roll estimations
<img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/estimates.jpg" width="600" height="300">

## Evaluation of LQR controller in Real Time Operating System
### ROLL 
<img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/proj5plots.jpg" width="500" height="400"> 

### PITCH
<img src="https://github.com/karthiknagarajansundar/embedded-control-system-for-quadrotor/blob/main/Images/proj5plots2.jpg" width="500" height="400">

## Conclusion
The controller has been simulated with different scenarios and verified that the controller is able to adapt to the changes accordingly and smoothly with faster response time. Hence the performance is observed to be good and optimal.

## License
[MIT License](https://github.com/karthiknagarajansundar/image-generation-using-GAN/blob/main/LICENSE)
