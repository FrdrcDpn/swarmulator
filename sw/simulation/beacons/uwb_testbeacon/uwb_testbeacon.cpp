#include "uwb_testbeacon.h"
#include "agent.h"
#include "main.h"

using namespace std;
uwb_testbeacon::uwb_testbeacon() {

}

void uwb_testbeacon::ranges_terminal(const uint16_t ID){
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

float twr_range(const uint16_t ID,const uint16_t ID_beacon ){
    float twr_range;

    return twr_range;
}

