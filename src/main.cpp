#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "Eigen/Dense"
#include "config.h"
#include "tools.hpp"
#include "datapoint.hpp"
#include "fusionukf.hpp"
#include "usagecheck.hpp"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


int main(int argc, char* argv[])
{
    
    /*Check the arguments provided by users*/
    check_arguments(argc, argv);
    
    string in_filename = argv[1];
    string out_filename = argv[2];
    
    ifstream in_file(in_filename.c_str(), ifstream::in); //c_str返回一个指向正规C字符串的指针常量，内容与本string串相同，返回const，防止用户修改
    ofstream out_file(out_filename.c_str(), ofstream::out);
    
    check_files(in_file, in_filename, out_file, out_filename);
    
    /*Read Data from file and store in memory*/
    vector<DataPoint> all_sensor_data;
    vector<DataPoint> all_truth_data;
    
    double val1, val2, val3;
    double x, y, vx, vy, v, yaw, yawrate;
    long long timestamp;
    string sensor_id;
    
    string line;
    
    while(getline(in_file, line)){
        
        istringstream iss(line);
        DataPoint sensor_data;
        DataPoint truth_data;
        
        iss >> sensor_id;
        if(sensor_id.compare("L") == 0){
            iss >> val1;
            iss >> val2;
            iss >> timestamp;
            
            VectorXd lidar_vec(LIDAR_DIM_);
            lidar_vec << val1, val2;
            sensor_data.set(timestamp, DataPointType::LIDAR, lidar_vec);
        } else if(sensor_id.compare("R") == 0){
            iss >> val1;
            iss >> val2;
            iss >> val3;
            iss >> timestamp;
            
            VectorXd radar_vec(RADAR_DIM_);
            radar_vec << val1, val2, val3;
            sensor_data.set(timestamp, DataPointType::RADAR, radar_vec);
        }
        
        //ground truth
        iss >> x;
        iss >> y;
        iss >> vx;
        iss >> vy;
        iss >> yaw;
        iss >> yawrate;
        
        v = sqrt(vx*vx + vy*vy);
        Eigen::VectorXd truth_vec(STATE_DIM_);
        truth_vec << x, y, v, yaw, yawrate;
        truth_data.set(timestamp, DataPointType::TRUTH, truth_vec);
        
        all_sensor_data.push_back(sensor_data);
        all_truth_data.push_back(truth_data);
    }
    
    /*Column names for output file*/
    out_file << "time_stamp" << "\t";
    out_file << "px_state" << "\t";
    out_file << "py_state" << "\t";
    out_file << "v_state" << "\t";
    out_file << "yaw_angle_state" << "\t";
    out_file << "yaw_rate_state" << "\t";
    out_file << "sensor_type" << "\t";
    out_file << "NIS" << "\t";
    out_file << "px_measured" << "\t";
    out_file << "py_measured" << "\t";
    out_file << "px_ground_truth" << "\t";
    out_file << "py_ground_truth" << "\t";
    out_file << "vx_ground_truth" << "\t";
    out_file << "vy_ground_truth" << "\n";
    
    /*use fusionufk for state estimations*/
    FusionUKF fusionUKF;
    
    vector<Eigen::VectorXd> predictions;
    vector<Eigen::VectorXd> ground_truths;
    vector<Eigen::VectorXd> estimations_vec;
    vector<Eigen::VectorXd> ground_truths_vec;
    
    Eigen::VectorXd prediction;
    Eigen::VectorXd measurement;
    Eigen::VectorXd truth;
    DataPointType sensor_type;
    DataPoint estimation;
    DataPoint sensor_data;
    string sensor_name;
    double nis;
    
    for(int k = 0; k < all_sensor_data.size(); k++){
        truth = all_truth_data[k].get(); //only raw data returned
        sensor_data = all_sensor_data[k]; //include timestamp
        timestamp = sensor_data.get_timestamp();
        
        sensor_type = sensor_data.get_type();
        sensor_name = ((sensor_type == DataPointType::LIDAR)? "lidar" : "radar" );
        measurement = sensor_data.get_state(); //position transfered from measurement
        
        /*predict next state using fusion ukf*/
        fusionUKF.process(sensor_data);
        prediction = fusionUKF.get();
        nis = fusionUKF.get_nis();
        
        /*write all info/Users/shaoguoliang/CarND-Project/UKFTest/src/datapoint.cpp in output file*/
        out_file << timestamp << "\t";
        out_file << prediction(0) << "\t";
        out_file << prediction(1) << "\t";
        out_file << prediction(2) << "\t";
        out_file << prediction(3) << "\t";
        out_file << prediction(4) << "\t";
        
        out_file << sensor_name << "\t";
        out_file << nis << "\t";
        out_file << measurement(0) << "\t";
        out_file << measurement(1) << "\t";
        
        out_file << truth(0) << "\t";
        out_file << truth(1) << "\t";
        out_file << truth(2) << "\t";
        out_file << truth(3) << "\n";
        
        /* stor all data in approriate vector for rmse calculation later*/
        estimation.set(timestamp, DataPointType::STATE, prediction);
        estimations_vec.push_back(estimation.get_state());
        predictions.push_back(prediction);
        
        ground_truths_vec.push_back(truth);
        ground_truths.push_back(all_truth_data[k].get_state());
        
    }
    
    /* calculate the rmse error*/
    Eigen::VectorXd RMSE;
    RMSE = calculate_RMSE(estimations_vec, ground_truths_vec);
    cout << "RMSE: " << endl << RMSE << endl;
    
    
    if(out_file.is_open()) {out_file.close(); }
    if(in_file.is_open()) {in_file.close(); }
    
    cout << "Done!" << endl;
    
	/* code */
	return 0;
}
