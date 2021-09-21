#include "UWBSIM_controller/ekf_state_estimator.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>

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

std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_int_distribution<int> uni(0,7); // guaranteed unbiased

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
  float xpos = s[ID]->get_state(0);
  float ypos = s[ID]->get_state(1);  
  pos={s[ID]->get_state(0), s[ID]->get_state(1), 0.f };
  speed={s[ID]->get_state(2), s[ID]->get_state(3), 0.f };
  ekf_range_set_state(&ekf,pos,speed);
  ekf.dt = simtime_seconds - simtime_seconds_store;
  simtime_seconds_store = simtime_seconds;
  ekf_range_predict(&ekf);
  
  // we have to update our ekf with different uwb measurements depending on the selected localisation scheme
  //tdoa, twr or different
  if(beacon_alg == "beacon_twr"){
  mtx_bcn.lock();

  //get the ranging and anchor data from our UWB dataset
  dist = UWB[ID].back()[0];
  anchor_0 ={UWB[ID].back()[1], UWB[ID].back()[2], 0.f };

  mtx_bcn.unlock();

  //input UWB measurements and update the estimate with new anchor (for now only anchor)
  ekf_range_update_dist_twr(&ekf,dist,anchor_0);
  
  }
  if(beacon_alg== "beacon_tdoa"){
  mtx_bcn.lock();
  //get the ranging data for our random beacon
  dist = UWB[ID].back()[0];
  //get the anchor data for our random anchor
  anchor_0 ={UWB[ID].back()[1], UWB[ID].back()[2], 0.f };
  anchor_1 ={UWB[ID].back()[3], UWB[ID].back()[4], 0.f };
  mtx_bcn.unlock();
  //get our UWB measurements and update the estimate with new anchor (for now only anchor)
  ekf_range_update_dist_tdoa(&ekf,dist,anchor_0, anchor_1);
  }

  //update our position and speed values
  pos = ekf_range_get_pos(&ekf);
  speed = ekf_range_get_speed(&ekf);

  //float zpos = 0;
  std::cout<<"predicted x: "<<pos.x<<" real x: "<< xpos <<" delta x: "<<pos.x-xpos<<" for agent: "<<ID<<std::endl;
  std::cout<<"predicted y: "<<pos.y<<" real y: "<< ypos <<" delta y: "<<pos.y-ypos<<" for agent: "<<ID<<std::endl;
  //std::cout<<"predicted z: "<<pos.z<<" real z: "<< zpos <<" delta z: "<<pos.z-zpos<<" for agent: "<<ID<<std::endl;
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
