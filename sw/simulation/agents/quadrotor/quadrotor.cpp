#include "quadrotor.h"
#include "trigonometry.h"
#include "draw.h"
#include "random"
#include "fmat.h"
#include "trigonometry.h"
#include "auxiliary.h"
#include "math.h"
#include <vector>
quadrotor::quadrotor(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;
  beacon->dynamic_beacon_init(ID); 
  orientation = 0.0;
  controller->set_saturation(1.0);
  state_desired = state;

  }

std::vector<float> quadrotor::state_update(std::vector<float> state)
{
  
  // y+ towards East
  beacon->dynamic_beacon_update(ID);
  beacon->measurement(ID);
  
  float v_x ;
  float v_y ;
  controller->get_velocity_command(ID, v_x, v_y);
  controller->saturate(v_x);
  controller->saturate(v_y);
  moving = controller->moving;

  //float vxr, vyr;
  //rotate_xy(v_x, v_y, orientation, vxr, vyr);
  
  // Acceleration
  state_desired.at(4) =  (v_x - state_desired[2]); // Acceleration x
  state_desired.at(5) =  (v_y - state_desired[3]); // Acceleration y
  // Velocity
  state_desired.at(2) += state_desired[4] * dt; // Velocity x
  state_desired.at(3) += state_desired[5] * dt; // Velocity y
  // Position
  state_desired.at(0) += state_desired[2] * dt + 0.5 * state_desired[4] * pow(dt, 2); // Position x
  state_desired.at(1) += state_desired[3] * dt + 0.5 * state_desired[5] * pow(dt, 2); // Position y

  
  DesiredX = state_desired[0];
  DesiredY = state_desired[1];
  DesiredZ = 0;

  // torque input for motors (Ux)
  float Kpx = 1.5; 
  float Kdx = 0.3;
  float Kix = 0.0001; 

  float errorSumXp = errorSumX + ( DesiredX - x );

  errorSumX = errorSumXp;
  float dDesiredXp = ( DesiredX - DesiredXpre ) / dt;
  dDesiredX = dDesiredXp;
  DesiredXpre = DesiredX;

  float Ux = Kpx*( DesiredX - x  ) + Kdx*( dDesiredX - dx ) + Kix*errorSumX;

  // torque input for motors (Uy)
  float Kpy = 1.5; 
  float Kdy = 0.3; 
  float Kiy = 0.0001; 

  errorSumY = errorSumY + ( DesiredY - y );

  dDesiredY = ( DesiredY - DesiredYpre ) / dt;
  DesiredYpre = DesiredY;

  float Uy = Kpy*(DesiredY - y  ) + Kdy*( dDesiredY - dy ) + Kiy*errorSumY;

  // torque input for motors (U1)
  float Kpz = 5; 
  float Kdz = 2; 
  float Kiz = 0.001; 

  errorSumZ = errorSumZ + ( DesiredZ - z );

  dDesiredZ = ( DesiredZ - DesiredZpre ) / dt;
  DesiredZpre = DesiredZ;

  float U1 = Kpz*( DesiredZ - z ) + Kdz*( dDesiredZ - dz ) + Kiz*errorSumZ; 

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


  // calculate U2
  float KpP = 5;
  float KdP = 1; 
  float KiP = 1;


  errorSumPhi = errorSumPhi + ( DesiredPhi - Phi );

  dDesiredPhi = ( DesiredPhi - DesiredPhipre ) / dt;
  DesiredPhipre = DesiredPhi;
  float U2 = KpP*( DesiredPhi - Phi ) + KdP*( dDesiredPhi - dPhi )  + KiP*errorSumPhi;
  
  // calculate U3
  float KpT = 5; 
  float KdT = 1; 
  float KiT = 1;

  errorSumTheta = errorSumTheta + ( DesiredTheta - Theta );

  dDesiredTheta = (DesiredTheta-DesiredThetapre)/dt;
  DesiredThetapre = DesiredTheta;

  float U3 = KpT*( DesiredTheta - Theta ) + KdT*( dDesiredTheta - dTheta ) + KiT*errorSumTheta;
  
  // calculate U4
  float KpS = 10; 
  float KdS = 1; 
  float KiS = 1;

  float U4 = KpS*( 0 - Psi ) + KdS*( 0 - dPsi );

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

  std::cout<<x<<std::endl;
  std::cout<<y<<std::endl;
  std::cout<<z<<std::endl;
  std::cout<<dx<<std::endl;
  std::cout<<dy<<std::endl;
  std::cout<<dz<<std::endl;
  std::cout<<Theta<<std::endl;
  std::cout<<Phi<<std::endl;
 
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
