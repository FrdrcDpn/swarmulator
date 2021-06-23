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
    beacon_twr();
    void ranges_terminal(const uint16_t ID);
    float measurement(const uint16_t ID,const uint16_t ID_beacon_0);
    float measurement(const uint16_t ID,const uint16_t ID_beacon_0,const uint16_t ID_beacon_1){return 0; };
    /**
    * @brief Return noisy value following ratio distribution of 2 known distributions
    *
    * @param value value to which noise is to be added
    * @param gamma value
    * @param ratio value
    */
    float add_ratio_noise(float value, const double gamma, const double ratio);
};


#endif //BEACON_TWR_H
