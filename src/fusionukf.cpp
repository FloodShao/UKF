//
//  fusionukf.cpp
//  UKF
//
//  Created by 邵国亮 on 12/4/19.
//

#include "fusionukf.hpp"

void FusionUKF::initialize(const DataPoint& data){
    /*
     this function initialize the first state for the fusion UKF
     */
    this->x_ = data.get_state();
    this->P_ = MatrixXd::Identity(STATE_DIM_, STATE_DIM_);
    this->timestamp_ = data.get_timestamp();
    this->initialized = true;
}

void FusionUKF::update(const DataPoint& data){
    
    VectorXd predicted_z;
    MatrixXd sigma_x;
    MatrixXd sigma_z;
    MatrixXd S;
    
    //get the dt from timestamp difference
    double dt = (data.get_timestamp() - this->timestamp_) / 1.e6;
    
    //state prediction
    //use sigma point of motion model
    this->statePredicter.process(this->x_, this->P_, dt);
    this->x_ = this->statePredicter.getx();
    this->P_ = this->statePredicter.getP();
    sigma_x = this->statePredicter.get_sigma();//for calculating the Mxz in measurement update
    
    //measurement update
    this->measurementUpdater.process(data.get_type(), sigma_x); //obtain Mxz, predicted_z and sigma_z
    predicted_z = this->measurementUpdater.getz();
    sigma_z = this->measurementUpdater.get_sigma_z();
    S = this->measurementUpdater.getS();
    
    //state update
    this->stateUpdater.process(this->x_, predicted_z, data.get(), this->P_, S, sigma_x, sigma_z);
    this->x_ = this->stateUpdater.getx();
    this->P_ = this->stateUpdater.getP();
    this->nis = this->stateUpdater.getnis();
    
    //update the current timestamp
    this->timestamp_ = data.get_timestamp();
    
}


void FusionUKF::process(const DataPoint& data){
    this->initialized? this->update(data) : this->initialize(data);
}

double FusionUKF::get_nis() const{
    return this->nis;
}

VectorXd FusionUKF::get() const{
    return this->x_;
}
