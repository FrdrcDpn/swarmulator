#include "UWBSIM_controller/ekf_state_estimator.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>


std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_int_distribution<int> uni(0,3); // guaranteed unbiased


#ifndef EKF_P0_POS
#define EKF_P0_POS 1.0f
#endif

#ifndef KF_P0_SPEED
#define EKF_P0_SPEED 1.0f
#endif

#ifndef EKF_Q
#define EKF_Q 4.0f
#endif

#ifndef EKF_R_DIST
#define EKF_R_DIST 0.1f
#endif

#ifndef EKF_R_SPEED
#define EKF_R_SPEED 0.1f
#endif

// Initialiser
ekf_state_estimator::ekf_state_estimator()
{
  initialized = false;
};

void ekf_state_estimator::init_ekf_filter()
{
  //we leave the z variables 0; only 2D simulation for now
  //initialise the ekf with 0 values
  ekf_range_init(&ekf, EKF_P0_POS, EKF_P0_SPEED,
      EKF_Q, EKF_R_DIST, EKF_R_SPEED, 0.1f);

  //update and set initial states
  pos={s[ID]->get_state(0), s[ID]->get_state(1), 0.f };
  speed={s[ID]->get_state(2), s[ID]->get_state(3), 0.f };
  ekf_range_set_state(&ekf,pos,speed);

  initialized = true;
  simtime_seconds_store = simtime_seconds;
}

void ekf_state_estimator::run_ekf_filter()
{
  //update and set states
  pos={s[ID]->get_state(0), s[ID]->get_state(1), 0.f };
  speed={s[ID]->get_state(2), s[ID]->get_state(3), 0.f };
  ekf_range_set_state(&ekf,pos,speed);
  ekf.dt = simtime_seconds - simtime_seconds_store;
  simtime_seconds_store = simtime_seconds;
  ekf_range_predict(&ekf);
  auto random_beacon = uni(rng);
  std::cout<<random_beacon<<std::endl;
  //for now update using random anchor 0-3 
  mtx_bcn.lock();
  dist = UWB[ID][random_beacon].back()[0];
  mtx_bcn.unlock();
  anchor ={environment.uwb_beacon[random_beacon][0], environment.uwb_beacon[random_beacon][1], 0.f };

  //get our UWB measurements and update the estimate with new anchor (for now only anchor)
  ekf_range_update_dist(&ekf,dist,anchor);

  //update our position and speed values
  pos = ekf_range_get_pos(&ekf);
  speed = ekf_range_get_speed(&ekf);
  
  std::cout<<"predicted x pos "<<pos.x<<"real x pos "<< s[ID]->get_position(0) <<std::endl;
  std::cout<<"predicted y pos "<<pos.y<<"real y pos "<< s[ID]->get_position(1) <<std::endl;
  std::cout<<"predicted z pos "<<pos.y<<"real z pos "<< s[ID]->get_position(1) <<std::endl;
}

void ekf_state_estimator::run(uint16_t ID_in)
{
  if (!initialized) {
    ID = ID_in;
    init_ekf_filter();
   // std::cout<<"Running ekf for agent ID: "<<ID<<std::endl;
  } else {
    run_ekf_filter();
  }
}
