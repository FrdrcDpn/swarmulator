#include "UWBSIM_controller/ekf_state_estimator.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>

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
//  ekf_range_init(&ekf, EKF_P0_POS, EKF_P0_SPEED,
    //  EKF_Q, EKF_R_DIST, EKF_R_SPEED, 0.1f);

  filterekf->init_ekf(0.1f, s[ID]->state[0], s[ID]->state[1], s[ID]->state[2], s[ID]->state[3], s[ID]->state[4], s[ID]->state[5]);
  // Initialise the EKF struct by setting the state to our estimate
  //pos={s[ID]->state[0], s[ID]->state[1], 0.f };
  //speed={s[ID]->state[2], s[ID]->state[3], 0.f };
  //ekf_range_set_state(&ekf,pos,speed);

  initialized = true;
  simtime_seconds_store = simtime_seconds;
}

void ekf_state_estimator::run_ekf_filter()
{
  float dt = simtime_seconds - simtime_seconds_store;
   // Initialise the EKF struct by setting the state to our estimate
 
  // Initialise the EKF struct by setting the state to our estimate
  // Initialise the EKF struct by setting the state to our estimate
  //pos={s[ID]->imu_state_estimate[0], s[ID]->imu_state_estimate[1], 0.f };
  //speed={s[ID]->imu_state_estimate[2], s[ID]->imu_state_estimate[3], 0.f };
  //ekf_range_set_state(&ekf,pos,speed);
  // prediction step
 //  std::cout<<"PREDICT"<<std::endl;
  filterekf->ekf_predict( ID, dt);
  // IMU update step
  //ekf_range_update_scalar(&ekf,s.at(ID)->imu_state_estimate[0], s.at(ID)->imu_state_estimate[1],0.0);

  // we have to update our ekf with different uwb measurements depending on the selected localisation scheme
  //tdoa, twr or different
  //  mtx_bcn.lock();
  //mtx_e.lock();
  // only perform update step when there are available UWB measurements 
  
  //if(param->enable_UWB() == 1 && beacon_measurement[ID].back()[0] == 1){
  if(param->enable_UWB() == 1 && s[ID]->UWBm[5] == 1){
  // prediction step
  
  // update step in case the beacon algorithm is TWR
  if(beacon_alg == "beacon_twr"){
 
  //get the ranging and anchor data from our UWB dataset
  
  float dist = abs(s[ID]->UWBm[0]);
 // anchor_0 ={s[ID]->UWBm[1], s[ID]->UWBm[2], 0.f };
 
  filterekf->ekf_update_twr(dist, s[ID]->UWBm[1], s[ID]->UWBm[2]);

  //input UWB measurements and update the estimate with new anchor (for now only anchor)
  //ekf_range_update_dist_twr(&ekf,dist,anchor_0);
  
  }

  // update step in case the beacon algorithm is TDOA
  if(beacon_alg== "beacon_tdoa"){
 
  //get the ranging and anchor data from our UWB dataset
 // float dist = UWB[ID].back()[0];
 // anchor_0 ={UWB[ID].back()[1], UWB[ID].back()[2], 0.f };
 // anchor_1 ={UWB[ID].back()[3], UWB[ID].back()[4], 0.f };
  float dist = s[ID]->UWBm[0];
 // anchor_0 ={s[ID]->UWBm[1], s[ID]->UWBm[2], 0.f };
  //anchor_1 ={s[ID]->UWBm[3], s[ID]->UWBm[4], 0.f };

  filterekf->ekf_update_tdoa(dist, s[ID]->UWBm[1], s[ID]->UWBm[2], s[ID]->UWBm[3], s[ID]->UWBm[4]);
  
  //input UWB measurements and update the estimate with anchors (for now only anchor)
//  / ekf_range_update_dist_tdoa(&ekf,dist,anchor_0, anchor_1);
  }
  s[ID]->UWBm.at(5) = 0;
  //UWB measuremend has been 'used', push back 0 to measurement vector
 // beacon_measurement[ID].push_back({0});
  
if(param->terminaloutput()==1.0){
  //output to terminal
  std::cout<<"BEACON UPDATE"<<std::endl;
}
 
  }

  // Write our measurements to the EKF estimate variable
  
  
 pos = filterekf->ekf_get_pos();
 speed = filterekf->ekf_get_speed();

  s.at(ID)->ekf_estimate[2] = speed.vx; // Velocity x
  s.at(ID)->ekf_estimate[3] = speed.vy; // Velocity y
  
  s.at(ID)->ekf_estimate[0] = pos.x; // Position x
  s.at(ID)->ekf_estimate[1] = pos.y; // Position y
 
  
  s.at(ID)->state_estimate[2] = speed.vx; // Velocity x
  s.at(ID)->state_estimate[3] = speed.vy; // Velocity y
  
  s.at(ID)->state_estimate[0] = pos.x; // Position x
  s.at(ID)->state_estimate[1] = pos.y; // Position y

  s.at(ID)->imu_state_estimate[2] = speed.vx; // Velocity x
  s.at(ID)->imu_state_estimate[3] = speed.vy; // Velocity y
  
  s.at(ID)->imu_state_estimate[0] = pos.x; // Position x
  s.at(ID)->imu_state_estimate[1] = pos.y; // Position y

  //Some operations to show interesting stuff in terminal
  float xpos = s[ID]->get_state(0);
  float ypos = s[ID]->get_state(1); 
  if(param->terminaloutput()==1.0){
  std::cout<<"predicted x: "<<pos.x<<" real x: "<< xpos <<" delta x: "<<pos.x-xpos<<" for agent: "<<ID<<std::endl;
  std::cout<<"predicted y: "<<pos.y<<" real y: "<< ypos <<" delta y: "<<pos.y-ypos<<" for agent: "<<ID<<std::endl;
  }
  simtime_seconds_store = simtime_seconds;
  //mtx_bcn.unlock();
}

void ekf_state_estimator::run(uint16_t ID_in)
{
  if (!initialized) {
    ID = ID_in;
    init_ekf_filter();
  } else {
    run_ekf_filter();
  }
}