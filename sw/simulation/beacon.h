#ifndef BEACON_H
#define BEACON_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

/**
 * Parent class defining a beacon. The dynamic implementation is handled in children classes.
 */

class Beacon
        {
public:
    /**
     * Constructor
     */
    Beacon();
    std::vector<float> range_beacon(const uint16_t ID);
    /**
     * Destructor
     */

    virtual void ranges_terminal(const uint16_t ID)=0;

    virtual ~Beacon() {};
};


#endif //BEACON_H
