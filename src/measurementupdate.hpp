//
//  measurementupdate.hpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef measurementupdate_hpp
#define measurementupdate_hpp

#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "tools.hpp"
#include "config.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class MeasurementUpdate{
    /*
     Calculate the predicted measurement z from the current state (predicted from the motion model)
     */
private:
    int z_dim_;
    DataPointType current_type;
    MatrixXd R_;
    MatrixXd S_; //covariance of the predicted measurement z
    VectorXd z_pred_;
    MatrixXd sigma_z;
    
    void init(const DataPointType sensor_type); //determine the R_, and z_dim_
    MatrixXd compute_sigma_z(const MatrixXd &sigma_x);
    VectorXd compute_z(const MatrixXd &sigma_z);
    MatrixXd compute_S(const MatrixXd &sigma_z, const VectorXd &predicted_z);
    
public:
    MeasurementUpdate(){}
    void process(const DataPointType sensor_type, const MatrixXd sigma_x);
    VectorXd getz() const;
    MatrixXd getS() const; //for calculating the kalman gain
    MatrixXd get_sigma_z() const; //for calculating the Sigma_xz of the numerator in kalman gain
    
};

#endif /* measurementupdate_hpp */
