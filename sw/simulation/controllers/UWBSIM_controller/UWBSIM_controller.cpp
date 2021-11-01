#include "UWBSIM_controller.h"
#include "main.h"
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

filter.run(ID,1);
//filter_estimate.run(ID,1);
float K = 1.5;
//s[ID]->state_ground.at(0) = filter.pos.x;
//s[ID]->state_ground.at(1) = filter.pos.y;

//s[ID]->state_ground.at(2) = filter.speed.x;
//s[ID]->state_ground.at(3) = filter.speed.y;

//run with estimated position
float current_x_location =filter.pos.x;
float current_y_location =filter.pos.y;
//float current_x_location =s[ID]->state[0];
//float current_y_location =s[ID]->state[1];
x_distance_to_waypoint = x_wp - current_x_location;
y_distance_to_waypoint = y_wp - current_y_location; 

//std::cout<<x_distance_to_waypoint<<"xdist"<<std::endl;
//std::cout<<y_distance_to_waypoint<<"ydist"<<std::endl;
if(sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.1&& wp_ID==0){
 x_wp = -1.5;
 y_wp = -1.5;
 wp_ID=1;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.1 && wp_ID==1){
 x_wp = 1.5;
 y_wp = -1.5;
 wp_ID=2;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.1 && wp_ID==2){
  x_wp = 1.5;
  y_wp = 1.5;
  wp_ID=3;
}else if (sqrt(x_distance_to_waypoint*x_distance_to_waypoint+y_distance_to_waypoint*y_distance_to_waypoint)<0.1 && wp_ID==3){
  x_wp = -1.5;
  y_wp = 1.5;
  wp_ID=0;
}

if (abs(x_distance_to_waypoint)> 0){
  v_x = K * x_distance_to_waypoint;
}else{
  v_x=0;
}


if (abs(y_distance_to_waypoint)> 0){
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
