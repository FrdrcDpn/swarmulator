#include "beacon.h"
#include "agent.h"
#include "main.h"
#include <random>
#include <vector>
#include <tgmath.h>
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

// input agent ID, output vector of ground truth of agent
std::vector<float> Beacon::ground_truth(const uint16_t ID) {
    float x, y;
    std::vector<float> ground_truth;
    x = s[ID]->get_position(0);
    y = s[ID]->get_position(1);
    ground_truth.push_back(x);
    ground_truth.push_back(y);
    return ground_truth;
}

float Beacon::add_gaussian_noise(float value, const double mean, const double stddev) {
    float noisy_value;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}

float Beacon::add_cauchy_noise(float value, const double mean, const double stddev) {
    float noisy_value = 0;
    return noisy_value;
}

