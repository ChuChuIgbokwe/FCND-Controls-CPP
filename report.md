### My Report

1. Implemented body rate control in C++.

```cpp
 momentCmd = V3F(Ixx, Iyy, Izz) * kpPQR * (pqrCmd - pqr);
 ```
2. Implement roll pitch control in C++.
```cpp
float normalizer = -collThrustCmd/mass;

if (collThrustCmd > 0.0) {
    // Roll 
    float b_x_actual = R(0, 2);
    float b_x_commanded = accelCmd.x/normalizer;
    float b_x_error = b_x_commanded - b_x_actual;
    float b_x_commanded_dot = kpBank * b_x_error;
    // Pitch
    float b_y_actual = R(1, 2);
    float b_y_commanded = accelCmd.y/normalizer;
    float b_y_error = b_y_commanded - b_y_actual;
    float b_y_commanded_dot = kpBank * b_y_error;
    // Transform to the body frame
    pqrCmd.x = (R(1, 0) * b_x_commanded_dot - R(0, 0) * b_y_commanded_dot) / R(2, 2);
    pqrCmd.y = (R(1, 1) * b_x_commanded_dot - R(0, 1) * b_y_commanded_dot) / R(2, 2);
    pqrCmd.z = 0;
  }
else {
    pqrCmd.x = 0.0;
    pqrCmd.y = 0.0;
    pqrCmd.z = 0.0;
    }
```
3. Implement altitude controller in C++.
```cpp
float b_z = R(2, 2);
float z_error = posZCmd - posZ;
float z_error_dot = velZCmd - velZ;

integratedAltitudeError += z_error * dt;
float u1_bar = kpPosZ * z_error + KiPosZ * integratedAltitudeError + kpVelZ * z_error_dot + accelZCmd;
float a = (u1_bar - CONST_GRAVITY)/b_z;
thrust = -mass * CONSTRAIN(a, -maxAscentRate / dt, maxAscentRate / dt);

```
4. Implement lateral position control in C++.
```cpp
  float x_error = posCmd.x - pos.x;
  float x_error_dot = CONSTRAIN(velCmd.x- vel.x, -maxSpeedXY, maxSpeedXY) ; 
  accelCmd.x = kpPosXY * x_error + kpVelXY * x_error_dot + accelCmdFF.x;
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);

  float y_error = posCmd.y - pos.y;
  float y_error_dot = CONSTRAIN(velCmd.y- vel.y, -maxSpeedXY, maxSpeedXY) ;
  accelCmd.y = kpPosXY * y_error + kpVelXY * y_error_dot + accelCmdFF.y;
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
 ```
Implement yaw control in C++.
```cpp
float yaw_error = yawCmd - yaw;
if (yaw_error > F_PI){
    yaw_error -= 2.f * F_PI;
}
else if (yaw_error < -F_PI){
    yaw_error += 2.f * F_PI;
}
yawRateCmd = kpYaw * yaw_error;
  ```
5. Implement calculating the motor commands given commanded thrust and moments in C++.
```cpp
float l = L / sqrt(2.0);
float c_bar = collThrustCmd;
float p_bar = momentCmd.x / l;
float q_bar = momentCmd.y / l;
float r_bar = momentCmd.z / -kappa;
cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) /4.0;
cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) /4.0;
cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) /4.0;
cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) /4.0;
```
#### Image Results
The resulting images from tuning the drone can be found [here](https://github.com/ChuChuIgbokwe/FCND-Controls-CPP/tree/master/pictures)

##### Challenges
* Where and when I chose to use the CONSTRAIN method affected my tuning. I reduced it's usage to the bare minimum and it made tuning easier.
* For Scenario 5 only Quuad 2 passes. I do not see any FAIL notice for quad 1 so I'm assuming only one quad needs to pass.
* The KpPosXY to KpVelXY was not 4