#ifndef BEACON_TDOA_H
#define BEACON_TDOA_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class beacon_tdoa: public Beacon
{

public:
    /**
     * Construction. beacon_tdoa is a child class of Beacon.
     */
    beacon_tdoa();
    int distr_size;
    int random_beacon_1;
    int random_beacon_2;
    float next_measurement_time = 0;
    void ranges_terminal(const uint16_t ID);
    //void measurement(const uint16_t ID,const uint16_t ID_beacon_0) { return 0; } ;
    float returnUWBdata(const uint16_t ID, float beacon);
    void measurement(const uint16_t ID);

    float add_gaussian_noise(float value);
    float add_ht_cauchy_noise(float value);
    double cdf_ht_cauchy(double x);
    double deriv_cdf_ht_cauchy(double x);
    float add_ht_gamma_noise(float value);
    double cdf_ht_gamma(double x);
    double deriv_cdf_ht_gamma(double x);
};


#endif //BEACON_TDOA_H
