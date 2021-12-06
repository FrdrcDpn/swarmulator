#ifndef UWBSIM_CONTROLLER_H
#define UWBSIM_CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "UWBSIM_controller/ekf_state_estimator.h"
#include "randomgenerator.h"
#include <fstream>


#define COMMAND_LOCAL 1

class UWBSIM_controller: public Controller
{
	std::ifstream inFile1;
	std::ifstream inFile2;
	std::ifstream inFile3;
	std::ifstream inFile4;
	std::ifstream inFile5;
	ekf_state_estimator filter;
	
public:
	float num_coordinates;
    int pos_id;
	float next_measurement_time;
	int wp_ID;
	float current_x_location;
	float current_y_location;
	float x_distance_to_waypoint;
	float y_distance_to_waypoint;
	float x_wp;
	float y_wp;
	float acc_x; 
	float acc_y;

	
	UWBSIM_controller();
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
};

#endif /*UWBSIM_CONTROLLER_H*/
