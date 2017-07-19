#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "math.h"
#include "measurement_package.h"
#include "FusionEKF.h"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


int main() {

	/*******************************************************************************
	 *  Set Measurements															 *
	 *******************************************************************************/
	vector<MeasurementPackage> measurement_pack_list;
        vector<VectorXd> ground_truth;

	// hardcoded input file with laser and radar measurements
	string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;
	// set i to get only first 3 measurments
	int i = 0;
	//while(getline(in_file, line) && (i<=3)){
	while(getline(in_file, line)){

		MeasurementPackage meas_package;

		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type;	//reads first element from the current line
		long timestamp;
		if(sensor_type.compare("L") == 0){	//laser measurement
			//Skip Radar measurements
			//continue;
                        if (i<1) cout << "NOTE (main): LIDAR" << endl << endl ;
			//read measurements
			//continue;
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x,y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);

		}else if(sensor_type.compare("R") == 0){
			//Skip Radar measurements
			continue;
                        if (i<1) cout << "NOTE (main): RADAR" << endl << endl ;
                        meas_package.sensor_type_ = MeasurementPackage::RADAR;
                        meas_package.raw_measurements_ = VectorXd(3);
                        float ro;
                        float theta;
                        float ro_dot;
                        iss >> ro;
                        iss >> theta;
                        iss >> ro_dot;
                        meas_package.raw_measurements_ << ro,theta, ro_dot;
                        iss >> timestamp;
                        meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
                        
		}
		i++;

                // debug
                float x_gt;
                float y_gt;
                float vx_gt;
                float vy_gt;
                iss >> x_gt;
                iss >> y_gt;
                iss >> vx_gt;
                iss >> vy_gt;
                VectorXd gt_values(4);
                gt_values(0) = x_gt;
                gt_values(1) = y_gt;
                gt_values(2) = vx_gt;
                gt_values(3) = vy_gt;

                ground_truth.push_back(gt_values);
	}

	//Create a Tracking instance
	//Tracking tracking;
        FusionEKF tracking;

        // used to compute the RMSE later
        Tools tools;
        vector<VectorXd> estimations;

        // intermediate CAM
        VectorXd estimate(4);

	//call the ProcessingMeasurement() function for each measurement
	size_t N = measurement_pack_list.size();
	for (size_t k = 0; k < N; ++k) {	//start filtering from the second frame (the speed is unknown in the first frame)
		tracking.ProcessMeasurement(measurement_pack_list[k]);
	    	
          double p_x = tracking.ekf_.x_(0);
          double p_y = tracking.ekf_.x_(1);
          double v1  = tracking.ekf_.x_(2);
          double v2  = tracking.ekf_.x_(3);

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;
         
          //cout << "NOTE (main): real values:" << endl << ground_truth[k] << endl ;
          //cout << "NOTE (main): est  values:" << endl << estimate  << endl ;

          estimations.push_back(estimate);
          //VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          //cout << "RMSE = " << endl << RMSE << endl;
	}


          VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
          cout << "RMSE = " << endl << RMSE << endl;

	if(in_file.is_open()){
		in_file.close();
	}
	return 0;
}

