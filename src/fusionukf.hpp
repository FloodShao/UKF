//
//  fusionukf.hpp
//  UKF
//
//  Created by 邵国亮 on 12/4/19.
//

#ifndef fusionukf_hpp
#define fusionukf_hpp

#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "statepredict.hpp"
#include "stateupdate.hpp"
#include "measurementupdate.hpp"
#include "config.h"
#include "datapoint.hpp"

class FusionUKF{
    
private:
    bool initialized = false;
    StatePredictor statePredicter;
    StateUpdater stateUpdater;
    MeasurementUpdate measurementUpdater;
    VectorXd x_ = VectorXd(STATE_DIM_);
    MatrixXd P_ = MatrixXd(STATE_DIM_, STATE_DIM_);
    long long timestamp_;
    double nis;
    
    void initialize(const DataPoint& data);
    void update(const DataPoint& data);
    
public:
    FusionUKF(){}
    void process(const DataPoint& data);
    double get_nis() const;
    VectorXd get() const;
};

#endif /* fusionukf_hpp */
