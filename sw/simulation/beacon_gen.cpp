#include "beacon_gen.h"
#include "main.h"
using namespace std;
// this class handles the broadcasting of the beacons

//constructor 
Beacon_gen::Beacon_gen(int i, std::vector<float> b)
{
  state_b = b;
  ID_b = i; 
 
}

// transmit a signal at defined beacon frequency defined in parameters file
// UWB signal length as defined in parameters file
std::vector<float> Beacon_gen::beacon_status_update(std::vector<float> state_b){
    // start of signal
    if(simtime_seconds>=next_measurement_time){
      end_measurement_time = simtime_seconds + param->UWB_signal_length();
      if(state_b[2]==1.0 && state_b[4] == 0.0){
        state_b[4] = 1.0;
        
        }
    }
    // end of signal, next signal after frequency interval
    if(simtime_seconds>=end_measurement_time){
      next_measurement_time = simtime_seconds + 1.0/state_b[3];
      if(state_b[2]==1.0 && state_b[4] == 1.0){
        state_b[4] = 0.0;
        
      }    
    }
  return state_b;
}


// if the beacon is a dynamic beacon, update the sate with the state estimate of the agent and the des state with the des traj of the agent
std::vector<float> Beacon_gen::beacon_dynamic_state_update(std::vector<float> state_b){
  
      if(state_b[5] == 1.0){
      state_b[0] = s[ID_b-8]->state_estimate[0];
      state_b[1] = s[ID_b-8]->state_estimate[1];
      state_b[7] = s[ID_b-8]->state[0]; 
      state_b[8] = s[ID_b-8]->state[1];
      state_b[9] = s[ID_b-8]->Cov[0];
      state_b[10] = s[ID_b-8]->Cov[3];
    }
  return state_b;
}

