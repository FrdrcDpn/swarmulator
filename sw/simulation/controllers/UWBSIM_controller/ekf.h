#ifndef EKF_H
#define EKF_H
#include "controller.h"
#include "Eigen/Dense"

struct pos
{
   float x;
   float y;
   float z;
};

struct speed
{
   float vx;
   float vy;
   float vz;
};

struct cov
{
   float c1;
   float c2;
   float c3;
   float c4;
   float c5;
   float c6;
};

class ekf
{
  // let's first define our matrices
private:
 


public:

 Eigen::MatrixXf X{6,1};
  Eigen::MatrixXf I{6,6};
  Eigen::MatrixXf Z{4,1};
  Eigen::MatrixXf Zm{4,1};
  Eigen::MatrixXf dfdx{6,6};
  Eigen::MatrixXf dfdu{6,6};
  Eigen::MatrixXf P{6,6};
  Eigen::MatrixXf Q{6,6};
  Eigen::MatrixXf R{4,4};
  Eigen::MatrixXf dhdn{4,4};
  Eigen::MatrixXf dhdx{4,6};
  Eigen::MatrixXf K{6,4};
  
  ekf();
  
  ~ekf() {};
  void init_ekf(float dt, float posx, float posy, float velx, float vely, float accx, float accy);

  void ekf_set_state();

  void ekf_get_state(); 

  struct pos ekf_get_pos(); 

  struct speed ekf_get_speed();
  struct cov ekf_get_cov();

  void ekf_predict(uint16_t ID, float dt);
  

  void ekf_update_acc(float ax, float ay);

  void ekf_update_twr(float dist, float anchor_x, float anchor_y);

  void ekf_update_tdoa(float dist, float anchor_0x, float anchor_0y, float anchor_1x, float anchor_1y); 
  void ekf_set_twr_noise(float s_twr);
   void ekf_set_tdoa_noise(float s_twr);
   void ekf_set_noise();
};

#endif /*EKF*/
