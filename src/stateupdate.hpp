//
//  stateupdate.hpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef stateupdate_hpp
#define stateupdate_hpp

#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "config.h"
#include "tools.hpp"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class StateUpdater{
    /*
     StateUpdater is a class responsible for updating the state vector x and state covariance matrix P(and current nis)
     */
private:
    VectorXd x_;
    MatrixXd P_;
    double nis;
    
    MatrixXd compute_Mxz(const VectorXd &predicted_x, const MatrixXd &sigma_x,
                         const VectorXd &predicted_z, const MatrixXd &sigma_z);
    
    void Update(const VectorXd &predicted_x, const VectorXd &predicted_z,
                    const VectorXd &measurement_z, const MatrixXd &Mxz,
                    const MatrixXd &predicted_P, const MatrixXd &S);
    
public:
    
    void process(const VectorXd &predicted_x, const VectorXd &predicted_z, const VectorXd &measurement_z, const MatrixXd &predicted_P, const MatrixXd &S, const MatrixXd &sigma_x, const MatrixXd &sigma_z);
    VectorXd getx() const;
    MatrixXd getP() const;
    double getnis() const;
    
};

#endif /* stateupdate_hpp */
