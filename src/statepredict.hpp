//
//  statepredict.hpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef statepredict_hpp
#define statepredict_hpp

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Eigen/Dense"
#include "config.h"
#include "tools.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class StatePredictor {

private:
    MatrixXd sigma_ = MatrixXd(STATE_DIM_, N_SP_); //predicted sigma points
    VectorXd x_ = VectorXd(STATE_DIM_); //predicted state vector
    MatrixXd P_ = MatrixXd(STATE_DIM_, STATE_DIM_);
    
    MatrixXd compute_augmented_sigma(const VectorXd &current_x, const MatrixXd &current_P);
    MatrixXd predict_sigma(const MatrixXd &augmented_sigma, double dt);
    VectorXd predict_x(const MatrixXd &predicted_sigma);
    MatrixXd predict_P(const MatrixXd &predicted_sigma, const VectorXd &predicted_x);

public:
    
    StatePredictor(){}
    void process(VectorXd &current_x, MatrixXd &current_P, double dt);
    MatrixXd get_sigma() const;
    MatrixXd getx() const;
    MatrixXd getP() const;
    
};

#endif /* predict_hpp */
