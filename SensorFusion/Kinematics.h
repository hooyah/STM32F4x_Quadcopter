/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include <math.h>

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

// This class is responsible for calculating vehicle attitude
uint8_t kinematicsType = 0;
float kinematicsAngle[3] = {0.0,0.0,0.0};
float gyroAngle[2] = {0.0,0.0};
float correctedRateVector[3] = {0.0,0.0,0.0};
float earthAccel[3] = {0.0,0.0,0.0};

float accelCutoff = 0.0;

void initializeBaseKinematicsParam(float hdgX, float hdgY) {
  for (uint8_t axis = XAXIS; axis <= ZAXIS; axis++)
    kinematicsAngle[axis] = 0.0;
  gyroAngle[XAXIS] = 0;
  gyroAngle[YAXIS] = 0;
}

void initializeKinematics(float hdgX, float hdgY);
void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,       
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel, 
                         float oneG,               float magX,          float magY,
                         float G_Dt);
float getGyroUnbias(uint8_t axis);
void calibrateKinematics();
 
  // returns the kinematicsAngle of a specific axis in SI units (radians)
//  const float getData(uint8_t axis) {
//    return kinematicsAngle[axis];
//  }
  // return heading as +PI/-PI
//  const float getHeading(uint8_t axis) {
//    return(kinematicsAngle[axis]);
//  }
  
#define M_PI		3.14159265358979323846
  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
const float kinematicsGetDegreesHeading(uint8_t axis) {
  float tDegrees;
    
  tDegrees = kinematicsAngle[axis]/M_PI*180.0;
  if (tDegrees < 0.0)
    return (tDegrees + 360.0);
  else
    return (tDegrees);
}
  
//  const uint8_t getType(void) {
    // This is set in each subclass to identify which algorithm used
//    return kinematicsType;
//  }


#endif

