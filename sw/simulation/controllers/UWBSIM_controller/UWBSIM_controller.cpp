#include "UWBSIM_controller.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>

#define SENSOR_MAX_RANGE 1.8
using namespace std;
UWBSIM_controller::UWBSIM_controller(): Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
  wp_ID = 0;
	x_wp = 0;
	y_wp = 0;
  }

//actual controller codea
void UWBSIM_controller::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{

float K = 1;
float current_x_location = s[ID]->get_position(0);
float current_y_location = s[ID]->get_position(1);

x_distance_to_waypoint = x_wp - current_x_location;
y_distance_to_waypoint = y_wp - current_y_location; 


if(sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.2 && wp_ID==0){
 x_wp = -10;
 y_wp = -10;
 wp_ID=1;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.2 && wp_ID==1){
 x_wp = 10;
 y_wp = -10;
 wp_ID=2;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.2 && wp_ID==2){
  x_wp = 10;
  y_wp = 10;
  wp_ID=3;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.2 && wp_ID==3){
  x_wp = -10;
  y_wp = 10;
  wp_ID=0;
}

if (abs(x_distance_to_waypoint)> 0.2){
  v_x = K * x_distance_to_waypoint;
}else{
  v_x=0;
}

if (abs(y_distance_to_waypoint)> 0.2){
  v_y = K * y_distance_to_waypoint;
}else{
  v_y=0;
}


  wall_avoidance_turn(ID, v_x, v_y, SENSOR_MAX_RANGE);
}


// animation of the controller and sensors
void UWBSIM_controller::animation(const uint16_t ID)
{

  draw d;
  d.circle_loop(SENSOR_MAX_RANGE);
 
}
