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
    void ranges_terminal(const uint16_t ID);
    float measurement(const uint16_t ID,const uint16_t ID_beacon_0) { return 0; } ;
    float measurement(const uint16_t ID,const uint16_t ID_beacon_0,const uint16_t ID_beacon_1);

    float add_gaussian_noise(float value);
    float add_ht_cauchy_noise(float value);
    double cdf_ht_cauchy(double x);
    double deriv_cdf_ht_cauchy(double x);
    float add_ht_gamma_noise(float value);
    double cdf_ht_gamma(double x);
    double deriv_cdf_ht_gamma(double x);
};


#endif //BEACON_TDOA_H
