#include "UWBSIM_controller.h"
#include "main.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>
#include <random>

#define SENSOR_MAX_RANGE 1.8
using namespace std;

UWBSIM_controller::UWBSIM_controller(): Controller()
{
  set_max_sensor_range(SENSOR_MAX_RANGE);
  wp_ID = 0;
	x_wp = 0;
	y_wp = 0;
  acc_x = 0;
  acc_y = 0;
  pos_id = 0;
  moving_1 = false;
  next_measurement_time = 0;
    inFile1.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory1.txt");
    if (!inFile1) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
  inFile2.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory2.txt");
    if (!inFile2) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }

}

//actual controller codea
void UWBSIM_controller::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  

if(ID == 0){
  // while trajectory file is not end of file yet
  if(!inFile2.eof())  // EOF is false here
  {
      inFile2 >> v_x >> v_y;     
      
  }else{
    moving_1 = true;
    v_x = -4;
    v_y = 4;
  }
}

if(ID == 1 && moving_1 == true){
  // while trajectory file is not end of file yet
  if(!inFile1.eof())  // EOF is false here
  {
      inFile1 >> v_x >> v_y;     
      
  }else{
    v_x = -1.5;
    v_y = -1.5;
    program_running = false;
  }
}

  std::cout<<v_x <<" desired X "<<v_y<<" Desired Y "<<std::endl;

  
  
std::cout<<"VELOCITY CYCLE"<<std::endl;
if(simtime_seconds>=next_measurement_time){
//update our state using IMU data (add noise on acceleration term)
  // Initialise our state without noise (1x)
  std::cout<<"IMU UPDATE"<<std::endl;
  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 gen(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::normal_distribution<float> dis(0, param->acc_noise_sigma());

    // Acceleration
  s[ID]->state_estimate.at(4) = s[ID]->state[4] + dis(gen); // Acceleration x
  s[ID]->state_estimate.at(5) = s[ID]->state[5] + dis(gen); // Acceleration y
  acc_x = s[ID]->state_estimate[4];
  acc_y = s[ID]->state_estimate[5];
  next_measurement_time = simtime_seconds + 1.0/param->IMU_frequency() ;
}else{
   // Acceleration
  s[ID]->state_estimate.at(4) = acc_x; // Acceleration x
  s[ID]->state_estimate.at(5) = acc_y; // Acceleration y
}
 
float timestep = 1.0/param->simulation_updatefreq();
  // Velocity
  s[ID]->state_estimate.at(2) += s[ID]->state_estimate[4] * timestep; // Velocity x
  s[ID]->state_estimate.at(3) += s[ID]->state_estimate[5] * timestep; // Velocity y

  // Position
  s[ID]->state_estimate.at(0)+= s[ID]->state_estimate[2]  * timestep + 0.5 * s[ID]->state_estimate[4]  * pow(timestep, 2); // Position x
  s[ID]->state_estimate.at(1) += s[ID]->state_estimate[3]  * timestep + 0.5 * s[ID]->state_estimate[5]  * pow(timestep, 2); // Position y
  

// if UWB is enabled and if a new measurement is available take update state estimate
if(param->enable_UWB() == 1 && beacon_measurement[ID].back()[0] == 1){
  filter.run(ID,1);
  s[ID]->state_estimate.at(2) = filter.speed.x; // Velocity x
  s[ID]->state_estimate.at(3) = filter.speed.y; // Velocity y
  
  // Position
  s[ID]->state_estimate.at(0) = filter.pos.x; // Position x
  s[ID]->state_estimate.at(1) = filter.pos.y; // Position y

  //measurement is 'used', push back 0 to measurement vector
  beacon_measurement[ID].push_back({0});
  std::cout<<"BEACON UPDATE"<<std::endl;

    

}


 // wall_avoidance_turn(ID, v_x, v_y, SENSOR_MAX_RANGE);
 }



// animation of the controller and sensors
void UWBSIM_controller::animation(const uint16_t ID)
{

  draw d;
  d.circle_loop(SENSOR_MAX_RANGE);
 
}
