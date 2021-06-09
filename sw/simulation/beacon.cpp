#include "beacon.h"
#include "agent.h"
#include "main.h"

#include <vector>
using namespace std;
Beacon::Beacon()
{

}

// input agent ID, output vector of ranges to beacon
std::vector<float> Beacon::range_beacon(const uint16_t ID){
    float b1, r1;
    std::vector<float> beacon_ranges;
    //draw the range from the beacons
    OmniscientObserver obs;
    for (uint16_t i = 1; i <= param->nbeacons(); i++) {
        obs.uwb_beacon(ID,r1,b1,i);
        beacon_ranges.push_back(r1);
    }
    return beacon_ranges;
}