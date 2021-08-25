#ifndef BEACON_TWR_H
#define BEACON_TWR_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class beacon_twr: public Beacon
{

public:

    /**
     * Construction. beacon_twr is a child class of Beacon.
     */
    
    std::vector<std::vector<std::vector<std::vector<float>>>> UWB;
    beacon_twr();
    void measurement2pos(const uint16_t ID);
    void ranges_terminal(const uint16_t ID);
    //void measurement();
    void measurement(const uint16_t ID);

    /**
    * @brief Return noisy value following ratio distribution of 2 known distributions
    *
    * @param value value to which noise is to be added
    * @param gamma value
    * @param ratio value
    */

    float add_gaussian_noise(float value);
    float add_ht_cauchy_noise(float value);
    float add_ht_gamma_noise(float value);
    double cdf_ht_gamma(double x);
    double deriv_cdf_ht_gamma(double x);
};


#endif //BEACON_TWR_H
