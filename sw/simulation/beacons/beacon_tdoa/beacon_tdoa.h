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
    float range(const uint16_t ID,const uint16_t ID_beacon );
    float add_summation_noise(float value, const double gamma, const double ratio);

};


#endif //BEACON_TDOA_H
