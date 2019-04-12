//
//  update.hpp
//  
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef update_hpp
#define update_hpp

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Eigen/Dense"
#include "tools.hpp"
#include "config.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Update{
    
private:
    int nz;
    DataPointType current_type;
    MatrixXd R;
    MatrixXd S;
    VectorXd z;
    MatrixXd sigma_z;
    
    void init(const DataPointType sensor_type);
    MatrixXd compute_sigma_z(const MatrixXd &sigma_x);
    MatrixXd compute_z(const MatrixXd &sigma_z);
    MatrixXd compute_S(const MatrixXd &sigma_z, const MatrixXd &predicted_z);
};


#endif /* update_hpp */
