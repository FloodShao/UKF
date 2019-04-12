//
//  measurementupdate.cpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#include "measurementupdate.hpp"


void MeasurementUpdate::init(const DataPointType sensor_type){ //determine the R_, and z_dim_
    
    this->current_type = sensor_type;
    if(sensor_type == DataPointType::LIDAR){
        this->z_dim_ = LIDAR_DIM_;
        this->R_ = MatrixXd(LIDAR_DIM_, LIDAR_DIM_);
        this->R_ << VAR_PX, 0.,
                    0., VAR_PY;
    } else if(sensor_type == DataPointType::RADAR){
        this->z_dim_ = RADAR_DIM_;
        this->R_ = MatrixXd(RADAR_DIM_, RADAR_DIM_);
        this->R_ << VAR_RHO, 0., 0.,
                    0., VAR_PHI, 0.,
                    0., 0., VAR_DRHO;
    }
    
}

MatrixXd MeasurementUpdate::compute_sigma_z(const MatrixXd &sigma_x){
    
    const double THRESHOLD = 1e-4;
    MatrixXd sigma_z = MatrixXd::Zero(this->z_dim_, N_SP_);
    
    for(int i = 0; i<N_SP_; i++){
        const double px = sigma_x(0, i);
        const double py = sigma_x(1, i);
        const double v = sigma_x(2, i);
        const double yaw = sigma_x(3, i);
        const double yawrate = sigma_x(4, i);
        
        if(this->current_type == DataPointType::LIDAR){
            sigma_z(0, i) = px;
            sigma_z(1, i) = py;
        } else if(this->current_type == DataPointType::RADAR){
            const double rho = sqrt(px*px + py*py);
            const double phi = atan2(py, px);
            const double vx = v*cos(yaw);
            const double vy = v*sin(yaw);
            const double drho = (rho > THRESHOLD)? (vx * px + vy * py)/rho : 0.;
            
            sigma_z(0, i) = rho;
            sigma_z(1, i) = phi;
            sigma_z(2, i) = drho;
        }
    }
    
    return sigma_z;
}

VectorXd MeasurementUpdate::compute_z(const MatrixXd &sigma_z){
    
    VectorXd z_pred = VectorXd::Zero(this->z_dim_);
    for(int i = 0; i<N_SP_; i++){
        z_pred += sigma_z.col(i) * WEIGHTS[i];
    }
    
    return z_pred;
}

MatrixXd MeasurementUpdate::compute_S(const MatrixXd &sigma_z, const VectorXd &z_pred){
    
    MatrixXd S = MatrixXd::Zero(this->z_dim_, this->z_dim_);
    for(int i = 0; i<N_SP_; i++){
        VectorXd diff = sigma_z.col(i) - z_pred;
        if(this->current_type == DataPointType::RADAR) diff(1) = normalize(diff(1));
        S += WEIGHTS[i] * diff * diff.transpose();
    }
    
    S += this->R_;
    return S;
    
}


void MeasurementUpdate::process(const DataPointType sensor_type, const MatrixXd sigma_x) {
    
    this->init(sensor_type);
    this->sigma_z =  this->compute_sigma_z(sigma_x);
    this->z_pred_ =  this->compute_z(this->sigma_z);
    this->S_ = this->compute_S(this->sigma_z, this->z_pred_);
    
}

VectorXd MeasurementUpdate::getz() const{
    return this->z_pred_;
}

MatrixXd MeasurementUpdate::getS() const{ //for calculating the kalman gain
    return this->S_;
}
MatrixXd MeasurementUpdate::get_sigma_z() const{ //for calculating the Sigma_xz of the numerator in kalman gain
    return this->sigma_z;
}
