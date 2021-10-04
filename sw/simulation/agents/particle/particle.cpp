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
    std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  //distanceuwb=beacon->returnUWBdata(ID, 1);
  float v_x = 0.0;
  float v_y = 0.0;
  controller->get_velocity_command(ID, v_x, v_y);
  controller->saturate(v_x);
  controller->saturate(v_y);
  moving = controller->moving;

  float vxr, vyr;
  rotate_xy(v_x, v_y, orientation, vxr, vyr);

  // Acceleration
  state_ground.at(4) = 2 * (vxr - state_ground[2]); // Acceleration x
  state_ground.at(5) = 2 * (vyr - state_ground[3]); // Acceleration y

  // Velocity
  state_ground.at(2) += state_ground[4] * dt; // Velocity x
  state_ground.at(3) += state_ground[5] * dt; // Velocity y

  // Position
  state_ground.at(0) += state_ground[2] * dt + 0.5 * state_ground[4] * pow(dt, 2); // Position x
  state_ground.at(1) += state_ground[3] * dt + 0.5 * state_ground[5] * pow(dt, 2); // Position y


  // Acceleration
  state.at(4) = 2 * (vxr - state[2])+dist(mt); // Acceleration x
  state.at(5) = 2 * (vyr - state[3])+dist(mt); // Acceleration y

  // Velocity
  state.at(2) += state[4] * dt; // Velocity x
  state.at(3) += state[5] * dt; // Velocity y

  // Position
  state.at(0) += state[2] * dt + 0.5 * state[4] * pow(dt, 2); // Position x
  state.at(1) += state[3] * dt + 0.5 * state[5] * pow(dt, 2); // Position y

  return state;
};

void particle::animation()
{
  draw d;
  d.circle(param->scale());
}
