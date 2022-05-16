#include "ekf.h"
#include "main.h"
#include <iostream>
#include <string>
#include <sstream>
#include "trigonometry.h"
#include <random>

#define PN_X 1.0f
#define PN_Y 1.0f
#define PN_VX 1.1f
#define PN_VY 1.1f
#define PN_AX 1.01f
#define PN_AY 1.01f

#define S_AX 1000.01f
#define S_AY 1000.01f


ekf::ekf()
{
kR = param->kR();
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
  result.c2 = P(0,3);
  result.c3 = P(3,3);
  result.c4 = P(3,0);
  result.c5 = P(4,4);
  result.c6 = P(5,5);
  return result; 
}


void ekf::init_ekf(float dt, float posx, float posy, float velx, float vely, float accx, float accy){
float s_tdoa = tdoa_noise; 
float s_twr =twr_noise;

float s_ax = param->Q(); 
float s_ay = param->Q(); 
// initalise our matrices
X << posx,
     velx, 
     accx, 
     posy, 
     vely,
     accy;

U << 0,
     0,
     0,
     0,
     0,
     0;

B << 0,
     0,
     1,
     0,
     0,
     1;

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
     0, 0, s_ax, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, s_ay;

R << s_tdoa*s_tdoa, 0, 0, 0,
        0, s_twr*s_twr, 0, 0,
        0, 0, S_AX*S_AX, 0,
        0, 0, 0, S_AY*S_AY;

dfdx << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0,0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

dfdu << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
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
NP << 0, 0, 
       0, 0;
}

void ekf::ekf_predict(uint16_t ID, float dt){

//std::cout<<"--------"<<std::endl;
//std::cout<<"ID "<<ID<<" ranging std "<<sqrtf(R(1,1))<<std::endl;




     float q = param->Q(); 



Q <<   (pow(dt,5)/20)*q,  (pow(dt,4)/8)*q,  (pow(dt,3)/6)*q, 0, 0, 0,
      (pow(dt,4)/8)*q, (pow(dt,1)/1)*q,  (pow(dt,2)/2)*q, 0, 0, 0,
      (pow(dt,3)/6)*q,  (pow(dt,2)/2)*q,q , 0, 0, 0,
     0, 0, 0, (pow(dt,5)/20)*q,  (pow(dt,4)/8)*q,  (pow(dt,3)/6)*q,
     0, 0, 0,  (pow(dt,4)/8)*q, (pow(dt,3)/3)*q ,  (pow(dt,2)/2)*q,
     0, 0, 0,  (pow(dt,3)/6)*q,  (pow(dt,2)/2)*q, q;
  /* 



 //dt = param->EKF_timestep();
Q <<   (pow(dt,4)/4)*q,  (pow(dt,3)/2)*q,  (pow(dt,2)/2)*q, 0, 0, 0,
      (pow(dt,3)/2)*q, (pow(dt,2))*q,  dt*q, 0, 0, 0,
      (pow(dt,2)/2)*q,  dt*q,q , 0, 0, 0,
     0, 0, 0, (pow(dt,4)/4)*q,  (pow(dt,3)/2)*q,  (pow(dt,2)/2)*q,
     0, 0, 0,  (pow(dt,3)/2)*q, (pow(dt,2))*q ,  dt*q,
     0, 0, 0,  (pow(dt,2)/2)*q,  dt*q, q;

*/
//std::cout<<P(0,0)<<std::endl;
//std::cout<<P(0,3)<<std::endl;
//std::cout<<P(3,3)<<std::endl;
//std::cout<<P(3,0)<<std::endl;
dfdx << 1, dt, dt*dt*0.5, 0, 0, 0,
        0, 1, dt, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, dt, dt*dt*0.5,
        0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 1;
 
// predict next state
// = ;//+ distributiond(generatord);
//
float accx = s[ID]->imu_state_estimate[4];
float accy = s[ID]->imu_state_estimate[5];

X(0,0) = X(0,0) + X(1,0)*dt + accx*dt*dt/2; 
X(1,0) = X(1,0) + accx*dt; 
X(2,0) = accx;
X(3,0) = X(3,0) + X(4,0)*dt + accy*dt*dt/2;
X(4,0) = X(4,0) +accy*dt;
X(5,0) = accy ;

P = dfdx * P * dfdx.transpose() +Q;


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
//outlier rejection based on Mahalonobis distance
float error = dist-norm;
float mah_distance = abs(error/sqrtf(R(1,1)));  
      
if(mah_distance < 50){
dhdx << 0, 0, 0, 0, 0, 0,
        dx/ norm, 0, 0, dy/ norm, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;


dhdxtwr << dx/ norm, 0, 0, dy/ norm, 0, 0;

dhdntwr << 0, 1, 0, 0;

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
///float gainx = (P(0,0)*(dx/norm))/((P(0,0)*pow(dx/norm,2) + P(3,3)*pow(dy/norm,2) + R(1,1)));
//float gainy = (P(3,3)*(dy/norm))/((P(0,0)*pow(dx/norm,2) + P(3,3)*pow(dy/norm,2) + R(1,1)));
// calculate Kalman Gain
K = P*(dhdx.transpose())/((P(0,0)*pow(dx/norm,2) + P(3,3)*pow(dy/norm,2) + R(1,1)));
//K(0,1) = gainx;
//K(3,1) = gainy;



// update the state
//X(0,0) = X(0,0) + gainx*(dist-norm);
//X(3,0) = X(3,0) + gainy*(dist-norm);

//update state covariance


//P(0,3) = (-(gainx*dy/norm))*P(0,3);
////P(3,0) = (-(gainy*dx/norm))*P(3,0);

// update the state
X = X + K*(Z-Zm);

//update state covariance
//P = (I-K*dhdx)*P;
P =(I-K*dhdx)*P*(I-K*dhdx).transpose() + K*R*K.transpose();
//P(0,0) = (1-(gainx*dx/norm))*P(0,0);
//P(3,3) = (1-(gainy*dy/norm))*P(3,3);
}}


void ekf::ekf_update_tdoa(float dist, float anchor_0x, float anchor_0y, float anchor_1x, float anchor_1y){

//std::cout<<"TDOA"<<std::endl;
float dx0 = X(0,0) - anchor_0x;
float dy0 = X(3,0) - anchor_0y;

float dx1 = X(0,0) - anchor_1x;
float dy1 = X(3,0) - anchor_1y;

float d0 = sqrtf(dx0*dx0 + dy0*dy0);
float d1 = sqrtf(dx1*dx1 + dy1*dy1);

if(d0 != 0.0f && d1 != 0.0f){
     

float norm = d1-d0;

//outlier rejection based on Mahalonobis distance
float error = abs(dist-norm);
float mah_distance = error/sqrtf(R(0,0));  

float anchordistancesq = sqrt(pow((anchor_0x-anchor_1x),2)+ pow((anchor_0y-anchor_1y),2)); 
float distancediffsq = sqrt(error); 


float errorBaseDistance = sqrtf(powf(dhdx(0,0), 2) + powf(dhdx(0,3), 2));
float errorDistance = fabsf(error / errorBaseDistance);

//if (errorDistance < 0.4) {
if(anchordistancesq > distancediffsq){
if(mah_distance < 50){
// build Jacobian of observation model for anchor i

dhdx << ((dx1 / d1) - (dx0 / d0)), 0, 0, ((dy1 / d1) - (dy0 / d0)), 0, 0,
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

     //std::cout<<mah_distance<<std::endl;   
// calculate Kalman Gain
// calculate Kalman Gain
//K = P*(dhdx.transpose())*((dhdx*P*(dhdx.transpose()) + dhdn*R*(dhdn.transpose())).inverse());


//
K = P*(dhdx.transpose())/((P(0,0)*pow( ((dx1 / d1) - (dx0 / d0)),2) + P(3,3)*pow(((dy1 / d1) - (dy0 / d0)),2) + R(0,0)));
// calculate Kalman Gain
//float gainx = (P(0,0)*( ((dx1 / d1) - (dx0 / d0))))/((P(0,0)*pow( ((dx1 / d1) - (dx0 / d0)),2) + P(3,3)*pow(((dy1 / d1) - (dy0 / d0)),2) + R(0,0)));
//float gainy = (P(3,3)*(((dy1 / d1) - (dy0 / d0))))/((P(0,0)*pow( ((dx1 / d1) - (dx0 / d0)),2) + P(3,3)*pow(((dy1 / d1) - (dy0 / d0)),2) + R(0,0)));

//K(0,0) = gainx;
//K(3,0) = gainy;
// update the state
//X(0,0) = X(0,0) + gainx*(dist-norm);
//X(3,0) = X(3,0) + gainy*(dist-norm);



//P(0,3) = (-(gainx*((dy1 / d1) - (dy0 / d0))))*P(0,3);
//P(3,0) = (-(gainy*((dx1 / d1) - (dx0 / d0))))*P(3,0);
//update state covariance
//P = (I-K*dhdx)*P;

// update the state
X = X + K*(Z-Zm);

//update state covariance
//P = (I-K*dhdx)*P;
P =(I-K*dhdx)*P*(I-K*dhdx).transpose() + K*R*K.transpose();
//P(0,0) = (1-(gainx*((dx1 / d1) - (dx0 / d0))))*P(0,0);
//P(3,3) = (1-(gainy*((dy1 / d1) - (dy0 / d0))))*P(3,3);

}
}}}


void ekf::ekf_set_twr_noise(float s_twr){
   //  R(0,0) = S_TDOA*S_TDOA;
    R(1,1) = s_twr*s_twr;

}


void ekf::ekf_set_tdoa_noise(float s_tdoa){
   //  R(0,0) = S_TDOA*S_TDOA;
    R(0,0) = s_tdoa*s_tdoa;


}

void ekf::ekf_set_noise(){
   R << tdoa_noise*tdoa_noise, 0, 0, 0,
        0, twr_noise*twr_noise, 0, 0,
        0, 0, S_AX*S_AX, 0,
        0, 0, 0, S_AY*S_AY;
}