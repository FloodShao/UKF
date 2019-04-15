//
//  usagecheck.hpp
//  UKF
//
//  Created by 邵国亮 on 15/4/19.
//

#ifndef usagecheck_hpp
#define usagecheck_hpp

#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <iomanip>
#include "datapoint.hpp"
#include "Eigen/Dense"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_arguments(int argc, char* argv[]);
void check_files(ifstream &in_file, string &in_nams, ofstream &out_file, string &out_name);
void print_data(const VectorXd &RMSE, const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truths, const vector<DataPoint> &all_sensor_data);

#endif /* usagecheck_hpp */
