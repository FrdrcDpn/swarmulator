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
    void test();
    /**
     * Destructor
     */
    virtual void beacontest()=0;
    virtual void test_range(const uint16_t ID)=0;
    virtual ~Beacon() {};
};


#endif //BEACON_H
