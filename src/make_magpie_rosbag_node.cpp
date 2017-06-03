//
//	Make ROSbags from MagPIE dataset
//	David Hanley, David Degenhardt, Alex Faustino
//	
//	make_magpie_rosbag_node.cpp
//	Entry point to the package that converts portions of the MagPIE dataset to ROSbags.
//
//	Options:
//
//
//	Usage:
//		
//

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Preamble -----------------------------------*/
/*-----------------------------------------------------------------------------*/
/*----------------- Defines --------------------*/
/*--------------- End Defines ------------------*/

/*------------------ Includes ------------------*/
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
// Include the ROS C++ APIs
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
/*---------------- End Includes ----------------*/

/*---------------- Globals ---------------------*/
/*-------------- End Globals -------------------*/

/*------------------ Classes -------------------*/
/*---------------- End Classes -----------------*/

/*----------------- Namespaces -----------------*/
using namespace std;
/*--------------- End Namespaces ---------------*/

/*------------------ Pragmas -------------------*/
/*---------------- End Pragmas -----------------*/

/*------------- Function Prototypes ------------*/
/*----------- End Function Prototypes ----------*/
/*-----------------------------------------------------------------------------*/
/*------------------------------ End Preamble ---------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*-------------------------------- Helpers ------------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*------------------------------ End Helpers ----------------------------------*/
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*----------------------------------- Main ------------------------------------*/
/*-----------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
	/*---------------- Initializations ----------------*/
	string accel_file;
	string gyro_file;
	string ground_truth_file;
	string mag_file;
	string trialnum;
	string directorypath;
	ifstream inAccel;
	ifstream inGyro;
	ifstream inGT;
	ifstream inMag;
	vector<double> timestamp_a;
	vector<double> timestamp_g;
	vector<double> timestamp_gt;
	vector<double> timestamp_m;
	vector<double> ax;
	vector<double> ay;
	vector<double> az;
	vector<double> gx;
	vector<double> gy;
	vector<double> gz;
	vector<double> gtx;
	vector<double> gty;
	vector<double> gtz;
	vector<double> quat_x;
	vector<double> quat_y;
	vector<double> quat_z;
	vector<double> quat_w;
	vector<double> mx;
	vector<double> my;
	vector<double> mz;
	int countera;
	int counterg;
	int counterm;
	int countergt;
	double timestamp;
	double vx;
	double vy;
	double vz;
	double qw;
	double qx;
	double qy;
	double qz;
	sensor_msgs::Imu imu_msg0 = sensor_msgs::Imu();
	sensor_msgs::Imu imu_msg1 = sensor_msgs::Imu();
	sensor_msgs::MagneticField mag_msg = sensor_msgs::MagneticField(); 
	geometry_msgs::PoseStamped gt_msg = geometry_msgs::PoseStamped();
	/*-------------- End Initializations --------------*/

	/*---------------- Initialize ROS -----------------*/
	// Annouce this program to the ROS master as a "node" called "vio_main"
	ros::init(argc, argv, "make_magpie_rosbag_node");
	/*-------------- End Initialize ROS ---------------*/

	/*------------------- Read Files ------------------*/
	// Set File Names and Directories
	cout << "----------------------------------------------\n";
	cout << "         Input MagPIE Trial Filename          \n";
	cout << "----------------------------------------------\n";
	cout << "Input directory path\n";
	cin >> directorypath;
	cout << "Input trial number\n";
	cin >> trialnum;
	accel_file = directorypath + "Output_accel" + trialnum + ".txt";
	gyro_file = directorypath + "Output_gyro" + trialnum + ".txt";
	ground_truth_file = directorypath + "Output_gt" + trialnum + ".txt";
	mag_file = directorypath + "Output_mag" + trialnum + ".txt";
	cout << "----------------------------------------------\n";
	cout << "       End Input MagPIE Trial Filename        \n";
	cout << "----------------------------------------------\n";

	// Open files
	inAccel.open(accel_file.c_str());
	inGyro.open(gyro_file.c_str());
	inGT.open(ground_truth_file.c_str());
	inMag.open(mag_file.c_str());

	// Check that all files are open
	if(!inAccel || !inGyro || !inGT || !inMag)
	{
		cerr << "Error: file could not be opened" << endl;
		return 0;
	}

	// Read Values: Accelerometer
	while (!inAccel.eof())
	{
		inAccel >> timestamp >> vx >> vy >> vz;
		timestamp_a.push_back(timestamp);
		ax.push_back(vx);
		ay.push_back(vy);
		az.push_back(vz);
	}
	
	// Read Values: Gyroscope
	while (!inGyro.eof())
	{
		inGyro >> timestamp >> vx >> vy >> vz;
		timestamp_g.push_back(timestamp);
		gx.push_back(vx);
		gy.push_back(vy);
		gz.push_back(vz);
	}

	// Read Values: Ground Truth
	while (!inGT.eof())
	{
		inGT >> timestamp >> vx >> vy >> vz >> qx >> qy >> qz >> qw;
		timestamp_gt.push_back(timestamp);
		gtx.push_back(vx);
		gty.push_back(vy);
		gtz.push_back(vz);
		quat_x.push_back(qx);
		quat_y.push_back(qy);
		quat_z.push_back(qz);
		quat_w.push_back(qw);
	}

	// Read Values: Magnetometer
	while (!inMag.eof())
	{
		inMag >> timestamp >> vx >> vy >> vz;
		timestamp_m.push_back(timestamp);
		mx.push_back(vx);
		my.push_back(vy);
		mz.push_back(vz);
	}

	// Close files
	inAccel.close();
	inGyro.close();
	inGT.close();
	inMag.close();
	/*----------------- End Read Files ----------------*/


	/*----------------- Start up node -----------------*/
	// Start the node resource managers (communication, time, etc)
	ros::start();
	/*--------------- End Start up node ---------------*/

	/*-------------- Set ROS bag location -------------*/
	cout << "----------------------------------------------\n";
	cout << "     Input Desired Location of Bag File       \n";
	cout << "----------------------------------------------\n";
	cout << "Input directory path\n";
	cin >> directorypath;
	cout << "----------------------------------------------\n";
	cout << "   End Input Desired Location of Bag File     \n";
	cout << "----------------------------------------------\n";
	/*------------ End Set ROS bag location -----------*/

	/*------------- Setup ROS bag to save -------------*/
	rosbag::Bag bag_out(directorypath + "magpie.bag",rosbag::bagmode::Write);
	/*----------- End Setup ROS bag to save -----------*/

	/*-------- Iterate through all MagPIE data --------*/
	// Initialize Counters
	countera = 0;
	counterg = 0;
	counterm = 0;
	countergt = 0;
	while(ros::ok())
	{
		imu_msg0.header.stamp = (ros::Time)(timestamp_a[countera]);
		imu_msg0.header.frame_id = "accel";
    		imu_msg0.linear_acceleration.x = ax[countera];
    		imu_msg0.linear_acceleration.y = ay[countera];
    		imu_msg0.linear_acceleration.z = az[countera];

		imu_msg1.header.stamp = (ros::Time)(timestamp_g[counterg]);
		imu_msg1.header.frame_id = "gyro";
    		imu_msg1.angular_velocity.x = gx[counterg];
    		imu_msg1.angular_velocity.y = gy[counterg];
    		imu_msg1.angular_velocity.z = gz[counterg];

		mag_msg.header.stamp = (ros::Time)(timestamp_m[counterm]);
		mag_msg.header.frame_id = "mag";
		mag_msg.magnetic_field.x = mx[counterm];
		mag_msg.magnetic_field.y = my[counterm];
		mag_msg.magnetic_field.z = mz[counterm];

		gt_msg.header.stamp = (ros::Time)(timestamp_gt[countergt]);
		gt_msg.header.frame_id = "groundtruth";
		gt_msg.pose.position.x = gtx[countergt];
		gt_msg.pose.position.y = gty[countergt];
		gt_msg.pose.position.z = gtz[countergt];
		gt_msg.pose.orientation.x = quat_x[countergt];
		gt_msg.pose.orientation.y = quat_y[countergt];
		gt_msg.pose.orientation.z = quat_z[countergt];
		gt_msg.pose.orientation.w = quat_w[countergt];

		bag_out.write("/imu/accel",(ros::Time)(timestamp_a[countera]),imu_msg0);
		bag_out.write("/imu/gyro",(ros::Time)(timestamp_g[counterg]),imu_msg1);
		bag_out.write("/magnetic_field",(ros::Time)(timestamp_m[counterm]),mag_msg);
		bag_out.write("/ground_truth",(ros::Time)(timestamp_gt[countergt]),gt_msg);

		countera++;
		counterg++;
		counterm++;
		countergt++;
		if (countera >= timestamp_a.size())
		{
			countera = timestamp_a.size()-1;
		}
		if (counterg >= timestamp_g.size())
		{
			counterg = timestamp_g.size()-1;
		}
		if (counterm >= timestamp_m.size())
		{
			counterm = timestamp_m.size()-1;
		}
		if (countergt >= timestamp_gt.size())
		{
			countergt = timestamp_gt.size()-1;
		}


		//Process ROS callbacks until receiving a SIGINT (ctrl-c)
		ros::spinOnce();
	}
	/*------ End Iterate through all MagPIE data ------*/


	bag_out.close();

	// Stop the node's resources
	ros::shutdown();

	return 0;
}
/*-----------------------------------------------------------------------------*/
/*--------------------------------- End Main ----------------------------------*/
/*-----------------------------------------------------------------------------*/
