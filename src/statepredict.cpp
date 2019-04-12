//
//  predict.cpp
//  UKF
//
//  Created by 邵国亮 on 11/4/19.
//

#include "statepredict.hpp"

MatrixXd StatePredictor::compute_augmented_sigma(const VectorXd &current_x, const MatrixXd &current_P){
    
    MatrixXd augmented_sigma = MatrixXd::Zero(N_AUGMENT_, N_SP_); // store the mean of sigma all the sigma points
    VectorXd augmented_x = VectorXd::Zero(N_AUGMENT_);
    MatrixXd augmented_P = MatrixXd::Zero(N_AUGMENT_, N_AUGMENT_);
    
    augmented_x.head(STATE_DIM_) = current_x;
    augmented_P.topLeftCorner(STATE_DIM_, STATE_DIM_) = current_P;
    augmented_P(STATE_DIM_, STATE_DIM_) = VAR_SPEED_NOISE; //process covariance matrix, btm right corner
    augmented_P(STATE_DIM_+1, STATE_DIM_+1) = VAR_YAWRATE_NOISE;
    
    const MatrixXd L = augmented_P.llt().matrixL(); //cholesky decomposition, and lower triangular matrix
    augmented_sigma.col(0) = augmented_x;
    
    for(int i = 1; i<N_AUGMENT_; i++){
        augmented_sigma.col(i) = augmented_x + SCALE * L.col(i);
        augmented_sigma.col(i+N_AUGMENT_) = augmented_x - SCALE * L.col(i);
    }
    
    return augmented_sigma;
    
}

MatrixXd StatePredictor::predict_sigma(const MatrixXd &augmented_sigma, double dt){
    
    /*
     pass each sigma point through the nonlinear motion model with noise,
     Remeber to add the noise part
     */
    
    const double THRESH = 0.001;
    MatrixXd predicted_sigma = MatrixXd(STATE_DIM_, N_SP_); //unstacked state
    
    for(int i = 0; i<N_SP_; i++){
        /*get current state, unstack each sigma point into state and motion noise*/
        const double px = augmented_sigma(0, i);
        const double py = augmented_sigma(1, i);
        const double speed = augmented_sigma(2, i);
        const double yaw = augmented_sigma(3, i);
        const double yawrate = augmented_sigma(4, i);
        const double speed_noise = augmented_sigma(5, i);
        const double yawrate_noise = augmented_sigma(6, i);
        
        
        /*predict the next state with noise*/
        const double cos_yaw = cos(yaw);
        const double sin_yaw = sin(yaw);
        const double dt2 = dt*dt;
        const double p_noise = 0.5*dt2 * speed_noise; //position noise, this should be a std
        const double yaw_noise = 0.5*dt2 * yawrate_noise; //yaw noise, this should be a  std
        
        double p_px, p_py;
        
        if(fabs(yaw) < THRESH){ //moving straight
            p_px = px + speed * cos_yaw * dt + p_noise * cos_yaw;
            p_py = py + speed * sin_yaw * dt + p_noise * sin_yaw;
        } else{
            /*CTRV model, use the integral function*/
            const double k = speed / yawrate;
            const double theta = yaw + yawrate * dt;
            p_px = px + k * (sin(theta) - sin(yaw)) + p_noise * cos_yaw;
            p_py = py + k * (-cos(theta) + cos(yaw)) + p_noise * sin_yaw;
        }
        
        predicted_sigma(0, i) = p_px;
        predicted_sigma(1, i) = p_py;
        predicted_sigma(2, i) = speed + speed_noise * dt;
        predicted_sigma(3, i) = yaw + yawrate * dt + yaw_noise;
        predicted_sigma(4, i) = yawrate + yawrate_noise * dt;
    }
    
    return predicted_sigma;
}

VectorXd StatePredictor::predict_x(const MatrixXd &predicted_sigma){
    /*
     Recombine the transformed sigmapoints into predicted belief.
     Use the weighted predicted_sigma
     */
    VectorXd predicted_x = VectorXd::Zero(STATE_DIM_);
    for(int i = 0; i<N_SP_; i++){
        predicted_x += predicted_sigma.col(i) * WEIGHTS[i];
    }
    return predicted_x;
}

MatrixXd StatePredictor::predict_P(const MatrixXd &predicted_sigma, const VectorXd &predicted_x){
    
    MatrixXd predicted_P = MatrixXd::Zero(STATE_DIM_, STATE_DIM_);
    
    for(int i = 0; i<N_SP_; i++){
        VectorXd diff = predicted_sigma.col(i) - predicted_x;
        diff(3) = normalize(diff(3));
        predicted_P += WEIGHTS[i] * diff * diff.transpose();
    }
    
    return predicted_P;
}


void StatePredictor::process(VectorXd &current_x, MatrixXd &current_P, double dt){
    
    MatrixXd augmented_sigma = compute_augmented_sigma(current_x, current_P);
    this->sigma_ = predict_sigma(augmented_sigma, dt);
    this->x_ = predict_x(this->sigma_);
    this->P_ = predict_P(this->sigma_, this->x_);
    
}
MatrixXd StatePredictor::get_sigma() const{
    return this->sigma_;
}
MatrixXd StatePredictor::getx() const{
    return this->x_;
}
MatrixXd StatePredictor::getP() const{
    return this->P_;
}


