//
//  tools.hpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#ifndef tools_hpp
#define tools_hpp

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths );
double normalize(const double a);

#endif /* tools_hpp */
