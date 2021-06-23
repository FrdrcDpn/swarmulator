#include "beacon_twr.h"
#include "agent.h"
#include "main.h"

using namespace std;
beacon_twr::beacon_twr() {

}

void beacon_twr::ranges_terminal(const uint16_t ID){
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

float beacon_twr::measurement(const uint16_t ID,const uint16_t ID_beacon_0){
    float x_0, y_0, dx0, dy0, d,d_noisy;
    x_0 = environment.uwb_beacon[ID_beacon_0][0];
    y_0 = environment.uwb_beacon[ID_beacon_0][1];

    dx0 = s[ID]->get_position(0) - x_0;
    dy0 = s[ID]->get_position(1) - y_0;

    d = sqrt(dx0*dx0 + dy0*dy0);

    d_noisy = add_gaussian_noise(d,0.0,0.1);
    return d_noisy;
}

float beacon_twr::add_ratio_noise(float value, const double mean, const double stddev) {
    float noisy_value;
    std::default_random_engine generator;
    std::cauchy_distribution<double> dist(mean,stddev);
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}

