#include "quadrotor.h"
#include "trigonometry.h"
#include "draw.h"
#include "random"
#include "fmat.h"
#include "trigonometry.h"
#include "auxiliary.h"
#include "math.h"
#include <vector>
#include <random>

quadrotor::quadrotor(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
 // beacon->dynamic_beacon_init(ID); 
  orientation = 0.0;
  controller->set_saturation(1.0);
  state_estimate = s;
  state_desired_traj = s;
  init = false; 
}
std::vector<float> quadrotor::state_update(std::vector<float> state)
{
  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 gen(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::normal_distribution<float> dis(0, param->noise_motor_sigma());

  // y+ towards East
  //beacon->dynamic_beacon_update(ID);
  beacon->measurement(ID);
  
  float v_x ;
  float v_y ;
  
  //controller->saturate(v_x);
  //controller->saturate(v_y);
  moving = controller->moving;
  
  controller->get_velocity_command(ID, v_x, v_y);
  //float vxr, vyr;
  //rotate_xy(v_x, v_y, orientation, vxr, vyr);

  //generate desired trajectory to fly with our quadrotor based on inputs of velocity controller


  //input desired position to our quadrotor eom
  //DesiredX = state_desired_traj[0];
  //DesiredY = state_desired_traj[1];
 //if(init == false){
 //  s[ID]->state_estimate[0] = 0;
//   s[ID]->state_estimate[1] = 0;
 //  s[ID]->state_estimate[2] = 0;
 //  s[ID]->state_estimate[3] = 0;
 //  init = true; 
 //}
  DesiredX = v_x; 
  DesiredY = v_y;
  DesiredZ = 0;

  std::cout<<DesiredX <<" desired X "<<DesiredY<<"Desired Y "<<std::endl;
  // torque input for motors (Ux) x direction
  float Kpx = 10; 
  float Kdx = 1;
  float Kix = 0; 

  float errorSumXp = errorSumX + ( DesiredX - x );

  errorSumX = errorSumXp;

  dDesiredX = ( DesiredX - DesiredXpre ) / dt;
  DesiredXpre = DesiredX;
  ddDesiredX = ( dDesiredX - dDesiredXpre ) / dt;
  dDesiredXpre = dDesiredX;

  float Ux = Kpx*( DesiredX - x ) + Kdx*( dDesiredX - dx ) + Kix*errorSumX +dis(gen);

  // torque input for motors (Uy) y direction
  float Kpy = 10; 
  float Kdy = 1;
  float Kiy = 0; 

  errorSumY = errorSumY + ( DesiredY - y);

  dDesiredY = ( DesiredY - DesiredYpre ) / dt;
  DesiredYpre = DesiredY;
  ddDesiredY = ( dDesiredY - dDesiredYpre ) / dt;
  dDesiredYpre = dDesiredY;

  float Uy = Kpy*(DesiredY - y  ) + Kdy*( dDesiredY - dy) + Kiy*errorSumY +dis(gen);

  // torque input for motors (U1) throttle
  float Kpz = 20; 
  float Kdz = 0.3; 
  float Kiz = 0.001; 

  errorSumZ = errorSumZ + ( DesiredZ - z );

  dDesiredZ = ( DesiredZ - DesiredZpre ) / dt;
  DesiredZpre = DesiredZ;

  float U1 = Kpz*( DesiredZ - z ) + Kdz*( dDesiredZ - dz ) + Kiz*errorSumZ + dis(gen); 
  // desired phi and theta

  if(isnan(wrapToPi_f(-(m*(Uy*cos(Psi) - Ux*sin(Psi)))/(U1*(pow(cos(Psi),2) + pow(sin(Psi),2)))))){
    DesiredPhi = -Uy/g; 
  }else{
    DesiredPhi = wrapToPi_f(-(m*(Uy*cos(Psi) - Ux*sin(Psi)))/(U1*(pow(cos(Psi),2) + pow(sin(Psi),2))));
  
  }

  if(isnan(wrapToPi_f( (m*(Ux*cos(Psi) + Uy*sin(Psi)))/(U1*(pow(cos(Psi),2) + pow(sin(Psi),2)))))){
    DesiredTheta = Ux/g;
  }else{
    DesiredTheta = wrapToPi_f( (m*(Ux*cos(Psi) + Uy*sin(Psi)))/(U1*(pow(cos(Psi),2) + pow(sin(Psi),2))));
   
  }


  // calculate U2 roll
  float KpP = 0.5;
  float KdP = 0.001; 
  float KiP = 0;


  errorSumPhi = errorSumPhi + ( DesiredPhi - Phi );

  dDesiredPhi = ( DesiredPhi - DesiredPhipre ) / dt;
  DesiredPhipre = DesiredPhi;
  float U2 = KpP*( DesiredPhi - Phi ) + KdP*( dDesiredPhi - dPhi )  + KiP*errorSumPhi+dis(gen);
  
  // calculate U3 pitch
  float KpT = 0.5; 
  float KdT = 0.001; 
  float KiT = 0;

  errorSumTheta = errorSumTheta + ( DesiredTheta - Theta );

  dDesiredTheta = (DesiredTheta-DesiredThetapre)/dt;
  DesiredThetapre = DesiredTheta;

  float U3 = KpT*( DesiredTheta - Theta ) + KdT*( dDesiredTheta - dTheta ) + KiT*errorSumTheta+dis(gen);
  // calculate U4 yaw
  float KpS = 5; 
  float KdS = 0.0001; 
  float KiS = 0;

  float U4 = KpS*( 0 - Psi ) + KdS*( 0 - dPsi )+dis(gen);
  // now lets solve the ODE's of the quadrotor

  // x direction
  ddx = (U1/m)*( Theta*cos(Psi) + Phi*sin(Psi) );
  dx = dx + ddx*dt;
  x = x + dx*dt;

  // y direction 
  ddy = (U1/m)*( Theta*sin(Psi) - Phi*cos(Psi) );

  dy = dy + ddy*dt;
  y = y + dy*dt;

  // z direction 
  ddz = (U1/m)-g;

  dz = dz + ddz*dt;
  z = z + dz*dt;

  // phi direction 
  ddPhi =(L/Jx)*U2;
  dPhi = dPhi + ddPhi*dt;
  Phi = Phi + dPhi*dt;

  // theta direction 
  ddTheta =(L/Jy)*U3;
  dTheta =dTheta +ddTheta*dt;
  Theta = Theta +dTheta*dt;

  //psi direction 
  ddPsi = (1/Jz)*U4;
  dPsi = dPsi + ddPsi*dt;
  Psi = Psi + dPsi*dt;
 
  // desired trajectory acceleration
  state_desired_traj.at(4) =  ddDesiredX; // Acceleration x
  state_desired_traj.at(5) =  ddDesiredY; // Acceleration y
  // desired trajectory velocity
  state_desired_traj.at(2) = dDesiredX; // Velocity x
  state_desired_traj.at(3) = dDesiredY; // Velocity y
  // desired trajectory position
  state_desired_traj.at(0) = DesiredX; // Position x
  state_desired_traj.at(1) = DesiredY; // Position y

  //update our states 
  //actual attained state of the quadrotor according to linearised dynamics

  // Acceleration
  state.at(4) = ddx; // Acceleration x
  state.at(5) = ddy; // Acceleration y
  // Velocity
  state.at(2) = dx; // Velocity x
  state.at(3) = dy; // Velocity y
  // Position
  state.at(0) = x; // Position x
  state.at(1) = y; // Position y

  
  return state;
}

void quadrotor::animation()
{
  draw d;
  d.circle(param->scale());
}
