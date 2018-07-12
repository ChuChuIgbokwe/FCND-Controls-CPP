#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;						
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float l = L / sqrt(2);
	float c_bar = collThrustCmd;
	float p_bar = momentCmd.x / l;
	float q_bar = momentCmd.y / l;
	float r_bar = momentCmd.z / -kappa;

	// cmd.desiredThrustsN[0] = CONSTRAIN((c_bar + p_bar + q_bar + r_bar) /4.0 , minMotorThrust, maxMotorThrust);
	// cmd.desiredThrustsN[1] = CONSTRAIN((c_bar - p_bar + q_bar - r_bar) /4.0, minMotorThrust, maxMotorThrust);
	// cmd.desiredThrustsN[2] = CONSTRAIN((c_bar + p_bar - q_bar - r_bar) /4.0, minMotorThrust, maxMotorThrust);
  // cmd.desiredThrustsN[3] = CONSTRAIN((c_bar - p_bar - q_bar + r_bar) /4.0, minMotorThrust, maxMotorThrust);

  cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) /4.0;
	cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) /4.0;
	cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) /4.0;
  cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar + r_bar) /4.0;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd; 
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
 momentCmd = V3F(Ixx, Iyy, Izz) * kpPQR * (pqrCmd - pqr);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)
  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB(); 
	  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  float acceleration_target = -collThrustCmd/mass;
  // Roll 
  float b_x_actual = R(0, 2);
  float b_x_commanded = CONSTRAIN(accelCmd.x/acceleration_target, -maxTiltAngle, maxTiltAngle);
  float b_x_error = b_x_commanded - b_x_actual;
  float b_x_commanded_dot = kpBank * b_x_error;
  // Pitch
  float b_y_actual = R(1, 2);
  float b_y_commanded = CONSTRAIN(accelCmd.y/acceleration_target, -maxTiltAngle, maxTiltAngle);
  float b_y_error = b_y_commanded - b_y_actual;
  float b_y_commanded_dot = kpBank * b_y_error;
  // Transform to the bnody frame
  pqrCmd.x = (R(1, 0) * b_x_commanded_dot - R(0, 0) * b_y_commanded_dot) / R(2, 2);
  pqrCmd.y = (R(1, 1) * b_x_commanded_dot - R(0, 1) * b_y_commanded_dot) / R(2, 2);
  pqrCmd.z = 0;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // this->integratedAltitudeError += (posZCmd - posZ)* dt;

  // velZCmd = CONSTRAIN(velZCmd, -1 * maxAscentRate, maxDescentRate);
  // float velCmd_bar = kpPosZ * (posZCmd - posZ)+ velZCmd;
  // float a_z_desire = accelZCmd + kpVelZ * (velCmd_bar - velZ) + KiPosZ * this->integratedAltitudeError;
  // thrust = mass * ( -a_z_desire + 9.81f) / R(2, 2);


  float b_z = R(2, 2);
  float z_error = posZCmd - posZ;
  float z_error_dot = velZCmd - CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
  integratedAltitudeError += z_error * dt;
  float u1_bar = kpPosZ * z_error + KiPosZ * integratedAltitudeError + kpVelZ * z_error_dot + accelZCmd;

  thrust = -mass * (u1_bar - CONST_GRAVITY)/b_z;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  
  V3F accelCmd = accelCmdFF;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////  
  // if (velCmd.mag() > maxSpeedXY)
  // {
	//   velCmd *= maxSpeedXY / velCmd.mag();
  // }
  // V3F VelXYCmd_bar = kpPosXY * (posCmd - pos) + velCmd;
  // accelCmd +=  + kpVelXY * (VelXYCmd_bar - vel);
  // if (accelCmd.mag() > maxAccelXY)
  // {
	//   accelCmd = accelCmd * maxAccelXY / accelCmd.mag();
  // }  
  // accelCmd.z = 0; 

  float x_error = posCmd.x - pos.x;
  float x_error_dot = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY) - vel.x; 
  accelCmd.x = kpPosXY * x_error + kpVelXY * x_error_dot + accelCmdFF.x;
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);

  float y_error = posCmd.y - pos.y;
  float y_error_dot = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY) - vel.y;
  accelCmd.y = kpPosXY * y_error + kpVelXY * y_error_dot + accelCmdFF.y;
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  // float yaw_error = yawCmd - yaw;
  // yaw_error = fmodf(yaw_error, 2 * M_PI);
  // if (yaw_error >  M_PI)
  // {
	//   yaw_error -= 2 * M_PI;
  // }
  // if (yaw_error < -M_PI)
  // {
	//   yaw_error += 2 * M_PI;
  // }
  // yawRateCmd = kpYaw * yaw_error;

  float yaw_error = yawCmd - yaw;
  if (yaw_error > F_PI){
    yaw_error -= 2.f * F_PI;
  }
  else if (yaw_error < -F_PI){
    yaw_error += 2.f * F_PI;
  }
  yawRateCmd = kpYaw * yaw_error;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}