#include "ekf.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>

#define PN_X 1.0f
#define PN_Y 1.0f
#define PN_VX 1.0f
#define PN_VY 1.0f
#define PN_AX 1.0f
#define PN_AY 1.0f

#define S_TDOA 0.025f
#define S_TWR 0.05f
#define S_AX 0.01f
#define S_AY 0.01f


ekf::ekf()
{

}

void ekf::ekf_set_state(){}

void ekf::ekf_get_state(){}

struct pos ekf::ekf_get_pos(){
  struct pos result;
  result.x = X(0,0);
  result.y = X(3,0);
  return result;
}


struct speed ekf::ekf_get_speed(){
  struct speed result;
  result.vx = X(1,0);
  result.vy = X(4,0);
  return result; 
}

struct cov ekf::ekf_get_cov(){
  struct cov result;
  result.c1 = P(0,0);
  result.c2 = P(1,1);
  result.c3 = P(2,2);
  result.c4 = P(3,3);
  result.c5 = P(4,4);
  result.c6 = P(5,5);
  return result; 
}

void ekf::init_ekf(float dt, float posx, float posy, float velx, float vely, float accx, float accy){

// initalise our matrices
X << posx,
     velx, 
     accx, 
     posy, 
     vely,
     accy;

Z << 0,
     0,
     0,
     0;
     
Zm <<0,
     0,
     0,
     0;

P << PN_X, 0, 0, 0, 0, 0,
     0, PN_VX, 0, 0, 0, 0,
     0, 0, PN_AX, 0, 0, 0,
     0, 0, 0, PN_Y, 0, 0,
     0, 0, 0, 0, PN_VY, 0,
     0, 0, 0, 0, 0, PN_AY;

Q << 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, S_AX*S_AX, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, S_AY*S_AY;

R << S_TDOA*S_TDOA, 0, 0, 0,
        0, S_TWR*S_TWR, 0, 0,
        0, 0, S_AX*S_AX, 0,
        0, 0, 0, S_AY*S_AY;

dfdx << 1, dt, dt*dt/2, 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, dt, dt*dt/2,
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 1;

dfdu << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1;

I <<    1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

dhdn << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

dhdx << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
}

void ekf::ekf_predict(const uint16_t ID, float dt){

//std::cout<<P<<std::endl;
//std::cout<<"-----------------"<<std::endl;
dfdx << 1, dt, dt*dt/2, 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, dt, dt*dt/2,
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 1;

// predict next state
X(2,0) = s[ID]->imu_state_estimate[4];
X(1,0) = X(1,0) + X(2,0)*dt; 
X(0,0) = X(0,0) + X(1,0)*dt + X(2,0)*dt*dt/2 ; 
X(5,0) = s[ID]->imu_state_estimate[5];
X(4,0) = X(4,0) +X(5,0)*dt;
X(3,0) = X(3,0) + X(4,0)*dt + X(5,0)*dt*dt/2;
//predict state covariance
P = dfdx * P * dfdx.transpose() + dfdu * Q * dfdu.transpose();

}

void ekf::ekf_update_acc(float ax, float ay){

// build Jacobian of observation model for anchor i


dhdx << 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 1;

// our twr measurement
Z << 0,
     0,
     ax,
     ay;

Zm << 0,
     0,
     X(2,0),
     X(5,0);

// calculate Kalman Gain
K = P*(dhdx.transpose())*((dhdx*P*(dhdx.transpose()) + dhdn*R*(dhdn.transpose())).inverse());

// update the state
X = X + K*(Z-Zm);

//update state covariance
P = (I-K*dhdx)*P;

}

void ekf::ekf_update_twr(float dist, float anchor_x, float anchor_y){
//std::cout<<"TWR"<<std::endl;
float dx = X(0,0) - anchor_x;
float dy = X(3,0) - anchor_y;

float norm = sqrtf(dx * dx + dy * dy);

// build Jacobian of observation model for anchor i

dhdx << 0, 0, 0, 0, 0, 0,
        dx/ norm, 0, 0, dy/ norm, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;

// our twr measurement
Z << 0,
     dist,
     0,
     0;

Zm << 0,
     norm,
     0,
     0;

// calculate Kalman Gain
K = P*(dhdx.transpose())*((dhdx*P*(dhdx.transpose()) + dhdn*R*(dhdn.transpose())).inverse());

// update the state
X = X + K*(Z-Zm);

//update state covariance
P = (I-K*dhdx)*P;
}


void ekf::ekf_update_tdoa(float dist, float anchor_0x, float anchor_0y, float anchor_1x, float anchor_1y){
//std::cout<<"TDOA"<<std::endl;
float dx0 = X(0,0) - anchor_0x;
float dy0 = X(3,0) - anchor_0y;

float dx1 = X(0,0) - anchor_1x;
float dy1 = X(3,0) - anchor_1y;

float d0 = sqrt(dx0*dx0 + dy0*dy0);
float d1 = sqrt(dx1*dx1 + dy1*dy1);
float norm = d1-d0;

// build Jacobian of observation model for anchor i

dhdx << (dx1 / d1 - dx0 / d0), 0, 0, (dy1 / d1 - dy0 / d0), 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;



// our tdoa measurement
Z << dist,
     0,
     0,
     0;

Zm << norm,
     0,
     0,
     0;


// calculate Kalman Gain
K = P*(dhdx.transpose())*(  (dhdx*P*(dhdx.transpose()) + dhdn*R*(dhdn.transpose())).inverse()  );

// update the state
X = X + K*(Z-Zm);

//update state covariance
P = (I-K*dhdx)*P;
}
