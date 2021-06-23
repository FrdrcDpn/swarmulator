#include "beacon_tdoa.h"
#include "agent.h"
#include "main.h"

using namespace std;
beacon_tdoa::beacon_tdoa() {

}

void beacon_tdoa::ranges_terminal(const uint16_t ID){
    //output ranges to all beacons in terminal
    float r1;
    //draw the range from the beacons
    cout << "agent id:"<< ID;
    for (uint16_t i = 0; i < param->nbeacons(); i++) {
        r1 = range_beacon(ID)[i];
        cout << "   range b"<< i+1 <<" "<< r1;
    }
    cout << endl;
}

float beacon_tdoa::measurement(const uint16_t ID,const uint16_t ID_beacon_0,const uint16_t ID_beacon_1){
    float x_0, y_0, x_1, y_1, dx0, dy0, dx1, dy1, d0, d1, dd, dd_noisy;

    x_0 = environment.uwb_beacon[ID_beacon_0][0];
    y_0 = environment.uwb_beacon[ID_beacon_0][1];
    x_1 = environment.uwb_beacon[ID_beacon_1][0];
    y_1 = environment.uwb_beacon[ID_beacon_1][1];

    dx0 = s[ID]->get_position(0) - x_0;
    dy0 = s[ID]->get_position(1) - y_0;

    dx1 = s[ID]->get_position(0) - x_1;
    dy1 = s[ID]->get_position(1) - y_1;

    d0 = sqrt(dx0*dx0 + dy0*dy0);
    d1 = sqrt(dx1*dx1 + dy1*dy1);
    dd = d1-d0;

    dd_noisy = add_gaussian_noise(dd,0.0,0.1);
    return dd_noisy;
}

float beacon_tdoa::add_summation_noise(float value, const double mean, const double stddev) {
    float noisy_value;

    std::default_random_engine generator;
    std::cauchy_distribution<double> dist(mean,stddev);
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}