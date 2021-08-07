#ifndef UWBSIM_CONTROLLER_H
#define UWBSIM_CONTROLLER_H

#include <stdio.h>
#include <iostream>
#include "controller.h"

class UWBSIM_controller: public Controller
{
public:
	UWBSIM_controller():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
};

#endif /*UWBSIM_CONTROLLER_H*/
