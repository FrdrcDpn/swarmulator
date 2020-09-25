#include "pfsm_exploration.h"
#include "draw.h"
#include "auxiliary.h"
#include <algorithm> // std::find

pfsm_exploration::pfsm_exploration(): t(4)
{
  policy = read_matrix<float>(param->policy());
  timelim = 5. * param->simulation_updatefreq();
  moving_timer = rg.uniform_int(0, timelim);
  vmean = 0.5; // Adjustment velocity
  moving = false;
  selected_action = 3;
  st = 100; // Init
}

void pfsm_exploration::action_motion(const int &selected_action, float r, float t, float &v_x, float &v_y)
{
  std::vector<float> ang = {-1.0, -0.7, -0.3, -0.1, 0.1, 0.3, 0.7, 1.0};
  v_x = vmean;
  v_y = ang[selected_action];
}

void pfsm_exploration::state_action_lookup(const int ID, uint state_index)
{
  std::vector<float> p = policy[state_index];
  selected_action = rg.discrete_int(p);
}

void pfsm_exploration::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  v_x = 0.0;
  v_y = 0.0;
  get_lattice_motion_all(ID, v_x, v_y);

  std::vector<bool> sensor; // 4 sensors
  std::vector<int> temp;
  t.assess_situation(ID, sensor, temp);
  if (st != bool2int(sensor) || moving_timer == 1) { // on state change
    st = bool2int(sensor);
#ifdef ESTIMATOR
    uint a = 0;
    if (moving) {a = selected_action + 1;}
    pr.update(ID, st, a);
#endif
    state_action_lookup(ID, st);
    float r, t;
    cart2polar(s[ID]->state[2], s[ID]->state[3], r, t);
    action_motion(selected_action, r, t, vx_ref, vy_ref);
  }

  increase_counter_to_value(moving_timer, timelim, 1);

  v_x += vx_ref;
  v_y += vy_ref;
  wall_avoidance_turn(ID, v_x, v_y);

  moving = true;

}

void pfsm_exploration::animation(const uint16_t ID)
{
  draw d;
  d.circle_loop(rangesensor);
}