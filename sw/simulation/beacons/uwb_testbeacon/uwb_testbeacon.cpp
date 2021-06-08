#include "uwb_testbeacon.h"
#include "agent.h"
#include "main.h"

using namespace std;
uwb_testbeacon::uwb_testbeacon() {

}

void uwb_testbeacon::beacontest() {
    cout << "dit is de testbeaconclass"<<endl;
}

void uwb_testbeacon::test_range(const uint16_t ID){

    //scrap code to test
    float b1, r1;
    float b2, r2;
    float b3, r3;
    float b4, r4;
    float b5, r5;
    float b6, r6;
    float b7, r7;
    float b8, r8;
    //draw the range from the beacons
    OmniscientObserver obs;

    if(param->nbeacons()==1){
        obs.uwb_beacon(ID,r1,b1,1);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1 <<endl;

    }

    if(param->nbeacons()==2){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2 <<endl;

    }

    if(param->nbeacons()==3){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3 <<endl;

    }

    if(param->nbeacons()==4){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        obs.uwb_beacon(ID,r4,b4,4);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3;
        cout << "   range b4:"<< r4 <<endl;

    }

    if(param->nbeacons()==5){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        obs.uwb_beacon(ID,r4,b4,4);
        obs.uwb_beacon(ID,r5,b5,5);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3;
        cout << "   range b4:"<< r4;
        cout << "   range b5:"<< r5<<endl;

    }

    if(param->nbeacons()==6){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        obs.uwb_beacon(ID,r4,b4,4);
        obs.uwb_beacon(ID,r5,b5,5);
        obs.uwb_beacon(ID,r6,b6,6);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3;
        cout << "   range b4:"<< r4;
        cout << "   range b5:"<< r5;
        cout << "   range b6:"<< r6<<endl;

    }

    if(param->nbeacons()==7){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        obs.uwb_beacon(ID,r4,b4,4);
        obs.uwb_beacon(ID,r5,b5,5);
        obs.uwb_beacon(ID,r6,b6,6);
        obs.uwb_beacon(ID,r7,b7,7);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3;
        cout << "   range b4:"<< r4;
        cout << "   range b5:"<< r5;
        cout << "   range b6:"<< r6;
        cout << "   range b7:"<< r7<<endl;

    }

    if(param->nbeacons()==8){
        obs.uwb_beacon(ID,r1,b1,1);
        obs.uwb_beacon(ID,r2,b2,2);
        obs.uwb_beacon(ID,r3,b3,3);
        obs.uwb_beacon(ID,r4,b4,4);
        obs.uwb_beacon(ID,r5,b5,5);
        obs.uwb_beacon(ID,r6,b6,6);
        obs.uwb_beacon(ID,r7,b7,7);
        obs.uwb_beacon(ID,r8,b8,8);
        cout << "agent id:"<< ID;
        cout << "   range b1:"<< r1;
        cout << "   range b2:"<< r2;
        cout << "   range b3:"<< r3;
        cout << "   range b4:"<< r4;
        cout << "   range b5:"<< r5;
        cout << "   range b6:"<< r6;
        cout << "   range b7:"<< r7;
        cout << "   range b8:"<< r8 <<endl;

    }
}
