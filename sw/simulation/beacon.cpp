#include "beacon.h"
#include "agent.h"
#include "main.h"
#include <random>
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


float Beacon::add_gaussian_noise(float value, const double mean, const double stddev) {
    float noisy_value;

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}

float Beacon::add_ht_cauchy_noise(float value, const double mean, const double stddev) {
    float noisy_value;

    std::default_random_engine generator;
    std::cauchy_distribution<double> dist(mean,stddev);
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}

