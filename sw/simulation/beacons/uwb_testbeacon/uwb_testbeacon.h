#ifndef UWB_TESTBEACON_H
#define UWB_TESTBEACON_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class uwb_testbeacon: public Beacon
{

public:

    /**
     * Construction. uwb_testbeacon is a child class of Beacon.
     */
    uwb_testbeacon();
    void beacontest();

    void test_range(const uint16_t ID);

};


#endif //UWB_TESTBEACON_H
