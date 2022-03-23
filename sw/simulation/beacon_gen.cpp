#include "beacon_gen.h"
#include "main.h"
using namespace std;
// this class handles the broadcasting of the beacons

//constructor 
Beacon_gen::Beacon_gen(int i, std::vector<float> b)
{
  state_b = b;
  ID_b = i; 
  next_measurement_time = simtime_seconds;
}

// transmit a signal at defined beacon frequency defined in parameters file
// UWB signal length as defined in parameters file
std::vector<float> Beacon_gen::beacon_status_update(std::vector<float> state_b){
    // start of signal
    /*
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
*/
    if(simtime_seconds>=next_measurement_time){
      //we end transmission after certain uwb signal length
      end_measurement_time = next_measurement_time + param->UWB_signal_length();

      //the interval in which we transmit a 1 is defined by the beacon frequency
      next_measurement_time = next_measurement_time + 1.0/state_b[3];
      if(state_b[2]==1.0){
        state_b[4] = 1.0;
        }
    }

    //if the signal length reaches its end before a new measurement, stop transmission until the next measurement
    if(simtime_seconds>=end_measurement_time){
      if(state_b[2]==1.0){
        state_b[4] = 0.0;
        }
    }
    
    if(1.0/state_b[3] < param->UWB_signal_length()){
      std::cout<<"WARNING: UWB signal length > beacon "<<ID_b+1<<" frequency causing continuous UWB transmission"<<std::endl;
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

