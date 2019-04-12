//
//  config.h
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef config_h
#define config_h

#include <stdlib.h>
#include <math.h>

enum class DataPointType {
    LIDAR, RADAR, STATE, TRUTH
};

const int RADAR_DIM_ = 3;
const int LIDAR_DIM_ = 2;
const int STATE_DIM_ = 5; //we are using CRTV model [x, y, v, yaw, yawrate]
const int N_AUGMENT_ = STATE_DIM_ + 2; //plus 2 motion noise value
const int N_SP_ = 2*N_AUGMENT_ + 1; //number of sigma points to select

//process noise standard deviations
const double STD_SPEED_NOISE = 0.9; // longitude acceleration in m/s^2
const double STD_YAWRATE_NOISE = 0.6; // yaw acceleration in rad/s^2
const double VAR_SPEED_NOISE = STD_SPEED_NOISE * STD_SPEED_NOISE;
const double VAR_YAWRATE_NOISE = STD_YAWRATE_NOISE * STD_YAWRATE_NOISE;

//RADAR measurement noise standard deviation
const double STD_RHO = 0.3; // m
const double STD_PHI = 0.03; // rad
const double STD_DRHO = 0.3; // m/s
const double VAR_RHO = STD_RHO * STD_RHO;
const double VAR_PHI = STD_PHI * STD_PHI;
const double VAR_DRHO = STD_DRHO * STD_DRHO;

//LIDAR measurement noise standard deviation
const double STD_PX = 0.15; //m
const double STD_PY = 0.15; //m
const double VAR_PX = STD_PX * STD_PX;
const double VAR_PY = STD_PY * STD_PY;

const int KAPPA = 3 - N_AUGMENT_; // parameter for tuning
const double SCALE = sqrt(KAPPA + N_AUGMENT_);
const double W = 0.5 / (double)(KAPPA + N_AUGMENT_);
const double W0 = (double)KAPPA / (double)(KAPPA + N_AUGMENT_);
const double WEIGHTS[N_SP_] = {W0, W, W, W, W, W, W, W, W, W, W, W, W, W, W}; //14 SP ALTOGETHER

#endif /* config_h */
