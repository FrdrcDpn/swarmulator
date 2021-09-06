#ifndef UWBSIM_CONTROLLER_H
#define UWBSIM_CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "UWBSIM_controller/ekf_state_estimator.h"
#include "randomgenerator.h"



#define COMMAND_LOCAL 1

class UWBSIM_controller: public Controller
{

	ekf_state_estimator filter;
	
public:
	int wp_ID;
	float x_distance_to_waypoint;
	float y_distance_to_waypoint;
	float x_wp;
	float y_wp;
	UWBSIM_controller();
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
};

#endif /*UWBSIM_CONTROLLER_H*/
