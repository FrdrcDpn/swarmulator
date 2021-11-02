#include "beacon_tdoa.h"
#include "agent.h"
#include "main.h"
#include <stdio.h>
//#include <Python.h>
//#include <pyhelper.h>
#include <boost/math/special_functions/gamma.hpp>
using namespace std;
#define EPSILON 0.001

beacon_tdoa::beacon_tdoa() {
beacon_alg = "beacon_tdoa";
next_measurement_time = simtime_seconds;
}

void beacon_tdoa::ranges_terminal(const uint16_t ID){
    //output ranges to all beacons in terminal
    float r1;
    //draw the range from the beacons
    cout << "agent id:"<< ID;
    for (uint16_t i = 0; i < 8; i++) {
        r1 = range_beacon(ID)[i];
        cout << "   range b"<< i+1 <<" "<< r1;
    }
    cout << endl;
}
//in parameters file, change params file to define the noise type
//noise_type 0 = no noise
//noise_type 1 = gaussian noise
//noise_type 2 = heavy tailed cauchy noise
//noise_type 3 = heavy tailed gamma noise

float beacon_tdoa::returnUWBdata(const uint16_t ID, float beacon){
   // function that returns the latest distance entry of uwb data
   mtx_bcn.lock();
   float dist = UWB[ID].back()[0];
   mtx_bcn.unlock();
   return dist;
}

void beacon_tdoa::measurement(const uint16_t ID){
    float x_0, y_0, x_1, y_1, dx0, dy0, dx1, dy1, d0, d1, dd;

    random_device rd;  // Will be used to obtain a seed for the random number engine
    mt19937 rng(rd()); // random-number engine used (Mersenne-Twister in this case)
    uniform_real_distribution<> dis(0.1*param->UWB_interval(), 2*0.1*param->UWB_interval());
    bool static_beacon_1 = false;
    bool dynamic_beacon_1 = false;
    bool static_beacon_2 = false;
    bool dynamic_beacon_2 = false;

    //thread safe vector operation to access UWB data
    mtx_bcn.lock();
    uniform_int_distribution<int> uni(0,7+dynamic_uwb_beacon.size()); // guaranteed unbiased
    mtx_bcn.unlock();

    //only take measurements on defined interval with noise
    if(simtime_seconds>=next_measurement_time){
    
    //take measurements from random beacons that are enabled
    int random_beacon_1 = uni(rng);
    bool beacon_1_selected = false; 
    int random_beacon_2 = uni(rng);
    bool beacon_2_selected = false; 

    // check whether the first random beacon is enabled, and whether dynamic or not (if using dynamic beacons is enabled)
    while(beacon_1_selected == false){
        if(random_beacon_1 >= 0 && random_beacon_1 <=7){
            if(environment.uwb_beacon[random_beacon_1][2]==0){
            random_beacon_1= uni(rng);
            }else if(environment.uwb_beacon[random_beacon_1][2]==1){
            beacon_1_selected = true;
            static_beacon_1 = true;
            }
        }
        if(random_beacon_1>7){
            mtx_bcn.lock();
            random_beacon_1 = random_beacon_1-8;
            if(dynamic_uwb_beacon[random_beacon_1][2]==0){
            random_beacon_1= uni(rng);
            }else if(dynamic_uwb_beacon[random_beacon_1][2]==1 && random_beacon_1 == ID){
                   if(s.size() == 1 ){
                    beacon_1_selected = true;
                    static_beacon_1 = true;
                    random_beacon_1 = 1;
                }
                else{
                    random_beacon_1= uni(rng);
                }
            }else if(dynamic_uwb_beacon[random_beacon_1][2]==1 && random_beacon_1 != ID){
            beacon_1_selected = true;
            dynamic_beacon_1 = true;
            std::cout<<"agent "<<ID<<" ranges with agent "<<random_beacon_1<<std::endl;
            }
           mtx_bcn.unlock(); 
        }
    }

    // check whether the second random beacon is enabled, and whether dynamic or not (if using dynamic beacons is enabled)
    while(beacon_2_selected == false){
        while(random_beacon_2 == random_beacon_1){
            random_beacon_2= uni(rng);
        }
        if(random_beacon_2 >= 0 && random_beacon_2 <=7){
            if(environment.uwb_beacon[random_beacon_2][2]==0){
            random_beacon_2= uni(rng);
            }else if(environment.uwb_beacon[random_beacon_2][2]==1){
            beacon_2_selected = true;
            static_beacon_2 = true;
            }
        }
        if(random_beacon_2>7){
            mtx_bcn.lock();
            random_beacon_2 = random_beacon_2-8;
            if(dynamic_uwb_beacon[random_beacon_2][2]==0){
            random_beacon_2= uni(rng);
            }else if(dynamic_uwb_beacon[random_beacon_2][2]==1 && random_beacon_2 == ID){
                   if(s.size() == 1 ){
                    beacon_2_selected = true;
                    static_beacon_2 = true;
                    random_beacon_2 = 2;
                }
                else{
                    random_beacon_2= uni(rng);
                }
            }else if(dynamic_uwb_beacon[random_beacon_2][2]==1 && random_beacon_2 != ID){
            beacon_2_selected = true;
            dynamic_beacon_2 = true;
            }
           mtx_bcn.unlock(); 
        }
    }

    // coordinates of the first beacon
    if(dynamic_beacon_1 == true){
        mtx_bcn.lock();
    x_0 = dynamic_uwb_beacon[random_beacon_1][0];
    y_0 = dynamic_uwb_beacon[random_beacon_1][1]; 
    mtx_bcn.unlock();
    }
    if(static_beacon_1 == true){
    x_0 = environment.uwb_beacon[random_beacon_1][0];
    y_0 = environment.uwb_beacon[random_beacon_1][1];
    }

    // coordinates of the second beacon
    if(dynamic_beacon_2 == true){
        mtx_bcn.lock();
    x_1 = dynamic_uwb_beacon[random_beacon_2][0];
    y_1 = dynamic_uwb_beacon[random_beacon_2][1]; 
    mtx_bcn.unlock();
    }
    if(static_beacon_2 == true){
    x_1 = environment.uwb_beacon[random_beacon_2][0];
    y_1 = environment.uwb_beacon[random_beacon_2][1];
    }
    
    // generate tdoa measurements
    dx0 = s[ID]->get_position(0) - x_0;
    dy0 = s[ID]->get_position(1) - y_0;

    dx1 = s[ID]->get_position(0) - x_1;
    dy1 = s[ID]->get_position(1) - y_1;

    d0 = sqrt(dx0*dx0 + dy0*dy0);
    d1 = sqrt(dx1*dx1 + dy1*dy1);
    dd = d1-d0;

    //initialise uwb dataset
    mtx_bcn.lock();
    UWB.push_back(std::vector<std::vector<float>>());
    UWB[ID].push_back(std::vector<float>());

    //add preferred noise on top of tdoa measurements
    if (param->noise_type() == 0){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        UWB[ID].push_back({dd,x_0, y_0,x_1,y_1, simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 1){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        UWB[ID].push_back({add_gaussian_noise(dd),x_0, y_0,x_1,y_1, simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 2){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        UWB[ID].push_back({add_ht_cauchy_noise(dd),x_0, y_0,x_1,y_1,simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 3){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        UWB[ID].push_back({add_ht_gamma_noise(dd),x_0, y_0,x_1,y_1, simtime_seconds});
        mtx_bcn.unlock();
    }
    }
    //next measurement time interval
    next_measurement_time = next_measurement_time + param->UWB_interval() + dis(rng) -0.1*param->UWB_interval();
    std::cout<<"dynamic beacon_1 "<<dynamic_beacon_1<<" static beacon_1 "<<static_beacon_1<<std::endl;
    std::cout<<"dynamic beacon_2 "<<dynamic_beacon_2<<" static beacon_2 "<<static_beacon_2<<std::endl;

}
   

float beacon_tdoa::add_gaussian_noise(float value) {
    float noisy_value;
    float mean = 0;
  
       // Random seed
    random_device rd;

    // Initialize Mersenne Twister pseudo-random number generator
    mt19937 gen(rd());
    std::normal_distribution<double> dis(mean, param->gauss_sigma());
    // Add Gaussian noise
    noisy_value = value + value*dis(gen);

    return noisy_value;
}

//our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::cdf_ht_cauchy(double x)
{
   // generate our random variable
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    //gaussian cdf
    float gauss = 0.5*(1+erf( x/(sqrt(2)*param->gauss_sigma()))) * (1-param->htc_ratio());
    //cauchy cdf
    float cauchy = (((1/M_PI) * atan(x/param->htc_gamma())) + 0.5) * param->htc_ratio();
    //return ht_cauchy cdf
    return gauss + cauchy - tmp;
}

// the derivative of our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::deriv_cdf_ht_cauchy(double x)
{
    float gauss = (1-param->htc_ratio())*(1/((sqrt(2*M_PI))*param->gauss_sigma()))*exp((-(x*x)/(2*param->gauss_sigma()*param->gauss_sigma())));
    //cauchy cdf
    float cauchy = (param->htc_gamma()*param->htc_ratio())/((M_PI*x*x)+(param->htc_gamma()*param->htc_gamma()*M_PI));
    //return ht_cauchy cdf
    return gauss + cauchy;
}

//our ht_cauchy noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_tdoa::add_ht_cauchy_noise(float value){
    // old python integration code
    // CPyInstance hInstance;
    // PyRun_SimpleString("import sys");
    // PyRun_SimpleString("sys.path.append('sw/simulation/beacons/beacon_tdoa')");
    // CPyObject pName = PyUnicode_FromString("noise");
    // CPyObject pModule = PyImport_Import(pName);
    // CPyObject pFunc = PyObject_GetAttrString(pModule, "getInteger");
    // CPyObject pValue = PyObject_CallObject(pFunc, NULL);

    double x = 0; //our starting estimate to solve the cdf using Newton Raphson's method
    double h = cdf_ht_cauchy(x) / deriv_cdf_ht_cauchy(x);
    while (abs(h) >= EPSILON)
    {
        h = cdf_ht_cauchy(x)/deriv_cdf_ht_cauchy(x);
        // x(i+1) = x(i) - f(x) / f'(x)
        x = x - h;
    }
    return value*x+value;
}

//our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::cdf_ht_gamma(double x)
{
    // generate our random variable
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    //our gamma cdf
    float gamma = 0.5*(1+erf( (x-param->htg_mu())/(sqrt(2)*param->gauss_sigma())))/(1+param->htg_scale());
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
        gamma += boost::math::gamma_p(param->htg_k(), param->htg_lambda()*x)* param->htg_scale()/(1+param->htg_scale());
    }
    return gamma - tmp;
}

//the derivative of our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::deriv_cdf_ht_gamma(double x)
{
    //our gamma cdf
    float gamma = (((1/(sqrt(2*M_PI)*param->gauss_sigma()))*exp((-(x-param->htg_mu())*(x-param->htg_mu()))
            /(2*param->gauss_sigma()*param->gauss_sigma())))/((1+param->htg_scale())*param->gauss_sigma()));
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
        gamma += boost::math::gamma_p_derivative(param->htg_k(), param->htg_lambda()*x)* param->htg_scale()/(1+param->htg_scale());
    }
    return gamma;
}

//our ht_gamma noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_tdoa::add_ht_gamma_noise(float value){
    double x = 0; //our starting estimate to solve the cdf using Newton Raphson's method
    double h = cdf_ht_gamma(x) / deriv_cdf_ht_gamma(x);
    while (abs(h) >= EPSILON)
    {
        h = cdf_ht_gamma(x)/deriv_cdf_ht_gamma(x);
        // x(i+1) = x(i) - f(x) / f'(x)
        x = x - h;
    }
    return value*x + value;
}


