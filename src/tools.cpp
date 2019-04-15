//
//  tools.cpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#include "tools.hpp"


VectorXd calculate_RMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths ) {
    
    VectorXd RMSE(5);
    int n = estimations.size();
    
    for(int i = 0; i<n; i++){
        VectorXd diff = estimations[i] - ground_truths[i];
        diff = diff.array() * diff.array();
        RMSE += diff;
    }
    
    RMSE /= (double)n;
    RMSE = RMSE.array().sqrt();
    
    return RMSE;
}


double normalize(const double a) {
    return (fabs(a) > M_PI) ? remainder(a, 2.*M_PI) : a;
    //fabs calculate the abs of a float
}
