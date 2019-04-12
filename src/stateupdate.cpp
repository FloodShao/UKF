//
//  stateupdate.cpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#include "stateupdate.hpp"

MatrixXd StateUpdater::compute_Mxz(const VectorXd &predicted_x, const MatrixXd &sigma_x,
                                   const VectorXd &predicted_z, const MatrixXd &sigma_z){
    
    int z_dim = predicted_z.size();
    MatrixXd Mxz = MatrixXd::Zero(STATE_DIM_, z_dim);
    
    VectorXd dx;
    VectorXd dz;
    
    for(int i = 0; i<N_SP_; i++){
        dx = sigma_x.col(i) - predicted_x;
        dx(3) = normalize(dx(3));
        
        dz = sigma_z.col(i) - predicted_z;
        if(z_dim == RADAR_DIM_) dz(1) = normalize(dz(1));
        
        Mxz += WEIGHTS[i] * dx * dz.transpose();
    }
    
    return Mxz;
}

void StateUpdater::Update(const VectorXd &predicted_x, const VectorXd &predicted_z,
                const VectorXd &measurement_z, const MatrixXd &Mxz,
                              const MatrixXd &predicted_P, const MatrixXd &S){
    
    MatrixXd K = Mxz * S.inverse();
    
    VectorXd dz = measurement_z - predicted_z;
    if(predicted_z.size() == RADAR_DIM_) dz(1) = normalize(dz(1));
    
    this->x_ = predicted_x + K * dz;
    this->P_ = predicted_P - K * Mxz.transpose();
    this->nis = dz.transpose() * S.inverse() * dz;
}

void StateUpdater::process(const VectorXd &predicted_x, const VectorXd &predicted_z, const VectorXd &measurement_z, const MatrixXd &predicted_P, const MatrixXd &S, const MatrixXd &sigma_x, const MatrixXd &sigma_z){
    
    MatrixXd Mxz = this->compute_Mxz(predicted_x, sigma_x, predicted_z, sigma_z);
    this->Update(predicted_x, predicted_z, measurement_z, Mxz, predicted_P, S);
    
}

VectorXd StateUpdater::getx() const{
    return this->x_;
}

MatrixXd StateUpdater::getP() const{
    return this->P_;
}

double StateUpdater::getnis() const{
    return this->nis;
}
