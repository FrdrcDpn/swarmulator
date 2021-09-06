#ifndef EKF_STATE_ESTIMATOR_H
#define EKF_STATE_ESTIMATOR_H
#include "controller.h"

extern "C" {
#include "ekf_range.h"
}

class ekf_state_estimator
{
  // The omniscient observer is used to simulate sensing the other agents.
  OmniscientObserver o;
  bool initialized;
  uint16_t ID;
  float simtime_seconds_store;

public:
  struct discrete_ekf_no_north ekf_rl;
  ekf_state_estimator();
  ~ekf_state_estimator() {};
  void init_ekf_filter();
  void run_ekf_filter();
  void run(uint16_t ID_in);
};

#endif /*EKF_STATE_ESTIMATOR_H*/
