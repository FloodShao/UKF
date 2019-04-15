//
//  datapoint.cpp
//  UKF
//
//  Created by 邵国亮 on 12/4/19.
//

#include "datapoint.hpp"


DataPoint::DataPoint(const long long timestamp, const DataPointType sensor_type, const VectorXd raw){
    
    this->timestamp_ = timestamp;
    this->sensor_type_ = sensor_type;
    this->raw_ = raw;
    this->initialized = true;
    
}

void DataPoint::set(const long long timestamp, const DataPointType sensor_type, const VectorXd raw){
    this->timestamp_ = timestamp;
    this->sensor_type_ = sensor_type;
    this->raw_ = raw;
    this->initialized = true;
}


VectorXd DataPoint::get() const{
    return this->raw_;
}

VectorXd DataPoint::get_state() const{
    
    VectorXd state(STATE_DIM_);
    
    if(this->sensor_type_ == DataPointType::LIDAR){
        double px = this->raw_(0);
        double py = this->raw_(1);
        state << px, py, 0., 0., 0.;
    } else if(this->sensor_type_ == DataPointType::RADAR){
        double rho = this->raw_(0);
        double phi = this->raw_(1);
        double drho = this->raw_(2);
        
        double px = rho * cos(phi);
        double py = rho * sin(phi);
        state << px, py, 0., 0., 0.;
    } else if(this->sensor_type_ == DataPointType::STATE){
        state = this->raw_;
    } else if(this->sensor_type_ == DataPointType::TRUTH){
        double px = this->raw_(0);
        double py = this->raw_(1);
        double vx = this->raw_(2);
        double vy = this->raw_(3);
        
        double v = sqrt(vx*vx + vy*vy);
        double yaw = atan2(vy, vx);
        state << px, py, v, yaw, 0.;
    }
    
    return state;
}
DataPointType DataPoint::get_type() const{
    return this->sensor_type_;
}
long long DataPoint::get_timestamp() const{
    return this->timestamp_;
}
