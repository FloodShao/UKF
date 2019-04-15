//
//  usagecheck.cpp
//  UKF
//
//  Created by 邵国亮 on 15/4/19.
//

#include "usagecheck.hpp"


void check_arguments(int argc, char* argv[]){
    //make sure the user provide the input file and output file
    string usage_instruction = "Usage Instructions: ";
    usage_instruction += argv[0];
    usage_instruction += "/path/to/input.txt output.txt";
    
    bool has_valid_args = false;
    
    if(argc == 1){
        cerr << usage_instruction << endl;
    } else if(argc == 2){
        cerr << "Please include an output file.\n" << usage_instruction << endl;
    } else if(argc == 3){
        has_valid_args = true;
    } else if(argc > 3){
        cerr << "Too many arguments.\n" << endl;
    }
    
    if(!has_valid_args){
        exit(EXIT_FAILURE);
    }
}


void check_files(ifstream &in_file, string &in_name, ofstream &out_file, string &out_name){
    
    if(!in_file.is_open()){
        cerr << "Cannot Open Input File : " << in_name << endl;
        exit(EXIT_FAILURE);
    }
    
    if(!out_file.is_open()){
        cerr << "Cannot Open Output File : " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}


void print_data(const VectorXd &RMSE, const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths, const vector<DataPoint> &all_sensor_data){
    
    cout << "-----------------------------------" << endl;
    cout << setw(15) << left << "RMSE : " << " | " << RMSE(0) << " | " << RMSE(1) << " | " << RMSE(2) << " | " << RMSE(3) << " | " << endl;
    cout << "-----------------------------------" << endl;
    
    for(int k = 0; k < all_sensor_data.size(); k++){
        cout << "----------------------------------" << endl;
        cout << "#" << k+1 << " : " << all_sensor_data[k].get_timestamp() << endl;
        cout << "----------------------------------" << endl;
        
        if(all_sensor_data[k].get_type() == DataPointType::LIDAR){
            VectorXd lidar_v = all_sensor_data[k].get();
            cout << setw(15) << left << "LIDAR:" << " | ";
            cout << setw(15) << left << "lidar_v(0)" << " | ";
            cout << setw(15) << left << "lidar_v(1)" << " | " << endl;
        } else if(all_sensor_data[k].get_type() == DataPointType::RADAR){
            VectorXd radar_v = all_sensor_data[k].get();
            cout << setw(15) << left << "RADAR:" << " | ";
            cout << setw(15) << left << radar_v(0) << " | ";
            cout << setw(15) << left << radar_v(1) << " | ";
            cout << setw(15) << left << radar_v(2) << " | " << endl;
        }
        
        cout << setw(15) << left << "PREDICTION: " << " | ";
        cout << setw(15) << left << estimations[k](0) << " | ";
        cout << setw(15) << left << estimations[k](1) << " | ";
        cout << setw(15) << left << estimations[k](2) << " | ";
        cout << setw(15) << left << estimations[k](3) << " | ";
        cout << setw(15) << left << estimations[k](4) << " | " << endl;
        
        cout << setw(15) << left << "TRUTH: " << " | ";
        cout << setw(15) << left << ground_truths[k](0) << " | ";
        cout << setw(15) << left << ground_truths[k](1) << " | ";
        cout << setw(15) << left << ground_truths[k](2) << " | ";
        cout << setw(15) << left << ground_truths[k](3) << " | ";
        cout << setw(15) << left << ground_truths[k](4) << " | " << endl;
    }
}