#include "particle.h"
#include "trigonometry.h"
#include "draw.h"
#include "random"

particle::particle(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  beacon->dynamic_beacon_init(ID); 
  orientation = 0.0;
  controller->set_saturation(1.0);
  state_ground = state;
 
  }

std::vector<float> particle::state_update(std::vector<float> state)
{
  
  // NED frame
  // x+ towards North
  // y+ towards East
  beacon->dynamic_beacon_update(ID);
  beacon->measurement(ID);

  //controller->UWB = beacon->UWB;
    //controller->UWB = beacon->UWB;
  
  // Random seed
    std::random_device rd;
    // Initialize Mersenne Twister pseudo-random number generator
    std::mt19937 gen(rd());
    std::normal_distribution<float> dis(100, 20);
  //distanceuwb=beacon->returnUWBdata(ID, 1);
  float v_x ;
  float v_y ;
  controller->get_velocity_command(ID, v_x, v_y);
  controller->saturate(v_x);
  controller->saturate(v_y);
  moving = controller->moving;

  //float vxr, vyr;
  //rotate_xy(v_x, v_y, orientation, vxr, vyr);
  
  // Acceleration
  state_ground.at(4) = 2 * (v_x - state_ground[2]); // Acceleration x
  state_ground.at(5) = 2 * (v_y - state_ground[3]); // Acceleration y

  state.at(4) = 2*(v_x - state[2])*dis(gen)/100;
  state.at(5) = 2*(v_y- state[3])*dis(gen)/100;

  
  // Velocity
  state_ground.at(2) += state_ground[4] * dt; // Velocity x
  state_ground.at(3) += state_ground[5] * dt; // Velocity y

  state.at(2) += state[4] * dt;//+dists(mt); // Velocity x
  state.at(3) += state[5] * dt;//+dists(mt); // Velocity y  
  // Position

  state_ground.at(0) += state_ground[2] * dt + 0.5 * state_ground[4] * pow(dt, 2); // Position x
  state_ground.at(1) += state_ground[3] * dt + 0.5 * state_ground[5] * pow(dt, 2); // Position y
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2);//+dist(mt);  // Velocity x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2);//+dist(mt); // Position y
  
  return state;
};

void particle::animation()
{
  draw d;
  d.circle(param->scale());
}
