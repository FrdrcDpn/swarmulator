#include "UWBSIM_controller/ekf_state_estimator.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>

//std::random_device rd;     // only used once to initialise (seed) engine
//std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
//std::uniform_int_distribution<int> uni(0,7); // guaranteed unbiased

// Initialiser
ekf_state_estimator::ekf_state_estimator()
{
  //simtime_seconds_store = simtime_seconds;
  initialized = false;
  
};

void ekf_state_estimator::init_ekf_filter()
{

  float dt = 0.001 ;
  filterekf.init_ekf(param->EKF_timestep(), s[ID]->state[0], s[ID]->state[1], s[ID]->state[2], s[ID]->state[3], s[ID]->state[4], s[ID]->state[5]);
  // Initialise the EKF struct by setting the state to our estimate
  //pos={s[ID]->state[0], s[ID]->state[1], 0.f };
  //speed={s[ID]->state[2], s[ID]->state[3], 0.f };
  //ekf_range_set_state(&ekf,pos,speed);

  initialized = true;
  simtime_seconds_store = simtime_seconds;


    tdoa_t = simtime_seconds; 
   tdoa_t_stored = simtime_seconds; 
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
  
  filterekf.ekf_predict(ID, dt);
  //filterekf.ekf_set_noise();
  // IMU update step
  //ekf_range_update_scalar(&ekf,s.at(ID)->imu_state_estimate[0], s.at(ID)->imu_state_estimate[1],0.0);

  // we have to update our ekf with different uwb measurements depending on the selected localisation scheme
  //tdoa, twr or different
  //  mtx_bcn.lock();
  //mtx_e.lock();
  // only perform update step when there are available UWB measurements 

  //lets make a copy of our UWB measurement vector, as this vector is continuously altered by the beacon threads
  std::vector<float> UWBm_0 = s[ID]->UWBm; 

  //if uwb is enabled and we have or a tdoa measurement or a twr available
  if(param->enable_UWB() == 1 && (UWBm_0[5] == 1 || UWBm_0[9]==1)){
  // prediction step
  
  // update step in case the beacon algorithm is TWR
  if(beacon_alg == "beacon_twr"){
 
  //get the ranging and anchor data from our UWB dataset
  float dist = abs(UWBm_0[0]);
  filterekf.ekf_update_twr(dist, UWBm_0[1], UWBm_0[2]);
  s[ID]->UWBm[5] = 0;
  }
   
  // update step in case the beacon algorithm is TDOA
  if(beacon_alg== "beacon_tdoa"){
 
  float dist = UWBm_0[0];
  filterekf.ekf_update_tdoa(dist, UWBm_0[1], UWBm_0[2], UWBm_0[3], UWBm_0[4]);
  s[ID]->UWBm[5] = 0;
  
  }

 // update step in case the beacon algorithm is TDOA
  if(beacon_alg== "beacon_hybrid_extra"){
 

 if(UWBm_0[5] == 1){
float dist = UWBm_0[0];
  filterekf.ekf_set_tdoa_noise(0.22);
  filterekf.ekf_update_tdoa(dist, UWBm_0[1], UWBm_0[2], UWBm_0[3], UWBm_0[4]);
  s[ID]->UWBm[5] = 0;
 // std::cout<<"TDOA"<<std::endl;
 }

 if(UWBm_0[9] == 1 && param->dynamic_beacons() == 1){

  
    //If we use the standard ranging no information exchange approach, the approach is 0 in the parameters file
    if(param->dynamic_cov_approach()==0){
       tdoa_t = simtime_seconds - tdoa_t_stored; 

    
    float dist = UWBm_0[6];
    filterekf.ekf_set_twr_noise(0.16);
    filterekf.ekf_update_twr(dist, UWBm_0[7], UWBm_0[8]);
 
 }

   // ****** fourth INTER AGENT RANGING APPROACH ****** 3 noise updating
    if(param->dynamic_cov_approach()==4){
  tdoa_t = simtime_seconds - tdoa_t_stored; 

    
    // get the current covariance matrix
    //cov = filterekf.ekf_get_cov(); 
    
    // update the twr measurement noise with the covariance values of the quadrotor 
    
   //float theta = (atan2f((s[ID]->state_estimate[1]-UWBm_0[8]),s[ID]->state_estimate[0]-UWBm_0[7]));
   float theta = (atan2f(s[ID]->state_estimate[1]-UWBm_0[8],(s[ID]->state_estimate[0]-UWBm_0[7])));

    //if(theta < 0){
    //  theta = theta + M_PI/2;
    //}
    
    //float theta = atan(UWBm_0[2]-pos.y/UWBm_0[1]-pos.x);

   // if(theta < 0){
     // theta = theta + M_PI/2;
    //}
    
    
    float variance = cosf(theta)*cosf(theta)*(abs(UWBm_0[10])) + sinf(theta)*sinf(theta)*(abs(UWBm_0[12])) +2*(abs(UWBm_0[11]))*sinf(theta)*cosf(theta); 
    float variance_total_add =  abs(variance);
   
    if(ID == 0){
    std::cout<<"theta: "<<theta*180/M_PI<<std::endl;
    std::cout<<"covx: "<<UWBm_0[10]<<std::endl;
    std::cout<<"covy: "<<UWBm_0[12]<<std::endl;}
    //std::cout<<sqrtf( (0.16*0.16)+variance_total_add)<<std::endl;

    filterekf.ekf_set_twr_noise(sqrtf((0.16*0.16)+ variance_total_add));//2*sqrtf(0.16)*(variance_total_add));
    
    //std::cout<<variance_total_add<<std::endl;
    float dist = UWBm_0[6];
    filterekf.ekf_update_twr(dist, UWBm_0[7], UWBm_0[8]);
    
     }
    
    
 s[ID]->UWBm[9] = 0;
 // std::cout<<"TWR"<<std::endl;
  }
  
  
  }

  // update step in case the beacon algorithm is TWR
  if(beacon_alg == "beacon_hybrid"){

    //if we perform twr with a dynamic  beacon
    if(UWBm_0[6] == 0){

    // ****** ZERO'th INTER AGENT RANGING APPROACH ****** 0 STUPID INTER AGENT RANGING
  
    //If we use the standard ranging no information exchange approach, the approach is 0 in the parameters file
    if(param->dynamic_cov_approach()==0){
       tdoa_t = simtime_seconds - tdoa_t_stored; 

     if ((tdoa_t > 0.1 && param->max_UWB_range()<30)|| param->max_UWB_range()>40){
    float dist = UWBm_0[0];
    filterekf.ekf_update_twr(dist, UWBm_0[1], UWBm_0[2]);
    std::cout<<"TWR"<<std::endl;
    }}

     // ****** fourth INTER AGENT RANGING APPROACH ****** 3 noise updating
    if(param->dynamic_cov_approach()==4){
  tdoa_t = simtime_seconds - tdoa_t_stored; 

     if ((tdoa_t > 0.1 && param->max_UWB_range()<30)|| param->max_UWB_range()>40){
    // get the current covariance matrix
    //cov = filterekf.ekf_get_cov(); 
    
    // update the twr measurement noise with the covariance values of the quadrotor 
    
    float theta = atan(UWBm_0[2]-pos.y/UWBm_0[1]-pos.x);

    if(theta < 0){
      theta = theta + M_PI/2;
    }


 
    float variance = cos(theta)*cos(theta)*UWBm_0[7] + sin(theta)*sin(theta)*UWBm_0[9] +2*UWBm_0[8]*sin(theta)*cos(theta); 
    float variance_total_add =  variance;

   
    //std::cout<<variance<<std::endl;
    // just a failsafe in case a covariance matrix is not yet inialised in the beginning of the simulation
    if(isnan(sqrt(variance_total_add))){
      variance_total_add = 0;
    }
    
    //std::cout<<sqrtf(0.025*0.025 + variance_total_add)<<std::endl;
    filterekf.ekf_set_twr_noise(twr_noise*twr_noise + variance_total_add);

    //std::cout<<variance_total_add<<std::endl;
    float dist = abs(UWBm_0[0]);
    filterekf.ekf_update_twr(dist, UWBm_0[1], UWBm_0[2]);
    std::cout<<"TWR"<<std::endl;

     }
    }
    
    if(param->dynamic_cov_approach()==5){
  tdoa_t = simtime_seconds - tdoa_t_stored; 

    if ((tdoa_t > 0.1 && param->max_UWB_range()<30)|| param->max_UWB_range()>40){
  

    // get the current covariance matrix
    //cov = filterekf.ekf_get_cov(); 
    
    // update the twr measurement noise with the covariance values of the quadrotor 
    pos = filterekf.ekf_get_pos();

    float theta = atan(UWBm_0[2]-pos.y/UWBm_0[1]-pos.x);

    if(theta < 0){
      theta = theta + M_PI/2;
    }

    float variance_x = abs(UWBm_0[7]);
    float variance_y = abs(UWBm_0[9]);
     
    //std::cout<<variance<<std::endl;
    // just a failsafe in case a covariance matrix is not yet inialised in the beginning of the simulation
    if(isnan(sqrt(variance_x))){
      variance_x = 0;
    }
      if(isnan(sqrt(variance_y))){
      variance_y = 0;
    }
    // update in x

    filterekf.ekf_set_twr_noise(twr_noise*twr_noise+ variance_x);
    float dist = abs(UWBm_0[0]*cos(theta));
    filterekf.ekf_update_twr(dist, UWBm_0[1], pos.y);
    
     // update in y
    filterekf.ekf_set_twr_noise(twr_noise*twr_noise+ variance_y);
    dist = abs(UWBm_0[0]*sin(theta));
    filterekf.ekf_update_twr(dist, pos.x,UWBm_0[2]);
    std::cout<<"TWR"<<std::endl;
      }
    }
    
    
    }
    

    
  // std::cout<<"hybrid twr"<<std::endl;
  }

  // if we perform tdoa with 2 static beacons
  if(UWBm_0[6] == 1){


  
  
  float dist = UWBm_0[0];
  filterekf.ekf_update_tdoa(dist, UWBm_0[1], UWBm_0[2], UWBm_0[3], UWBm_0[4]);
  //std::cout<<"TDOA"<<std::endl;
 // std::cout<<"hybrid tdoa"<<std::endl;
 std::cout<<"TDOA"<<std::endl;
 tdoa_t_stored = simtime_seconds; 
  }
  s[ID]->UWBm[5] = 0;
  }

  
  //UWB measuremend has been 'used', push back 0 to measurement vector
 // beacon_measurement[ID].push_back({0});
  
if(param->terminaloutput()==1.0){
  //output to terminal
  std::cout<<"BEACON UPDATE"<<std::endl;
}
 
  

  // Write our measurements to the EKF estimate variable

 pos = filterekf.ekf_get_pos();
 speed = filterekf.ekf_get_speed();

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

  //update our covariance vector for later analysis 
   cov = filterekf.ekf_get_cov(); 
   s.at(ID)->Cov[0] = cov.c1; 
   s.at(ID)->Cov[1] = cov.c2; 
   s.at(ID)->Cov[2] = cov.c3; 
   s.at(ID)->Cov[3] = cov.c4; 
   s.at(ID)->Cov[4] = cov.c5; 
   s.at(ID)->Cov[5] = cov.c6; 


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