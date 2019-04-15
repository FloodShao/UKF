//
//  datapoint.hpp
//  UKF
//
//  Created by 邵国亮 on 12/4/19.
//

#ifndef datapoint_hpp
#define datapoint_hpp

#include <stdio.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "config.h"


using Eigen::VectorXd;

class DataPoint{
    
private:
    long long timestamp_;
    DataPointType sensor_type_;
    VectorXd raw_;
    bool initialized = false;
    
public:
    DataPoint(){}
    DataPoint(const long long timestamp, const DataPointType sensor_type, const VectorXd raw);
    void set(const long long timestamp, const DataPointType sensor_type, const VectorXd raw);
    VectorXd get() const;
    VectorXd get_state() const;
    DataPointType get_type() const;
    long long get_timestamp() const;
    
    
};

#endif /* datapoint_hpp */
