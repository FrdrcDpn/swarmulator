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
    float dist = 10;
    return dist;
}
void beacon_tdoa::measurement(const uint16_t ID){
    float x_0, y_0, x_1, y_1, dx0, dy0, dx1, dy1, d0, d1, dd;
    uint16_t ID_beacon_0 = 1; //beacons to be selected for measurements
    uint16_t ID_beacon_1 = 2;
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

    if (param->noise_type() == 0){
        //return dd;
    }
    else if (param->noise_type() == 1){
       // return add_gaussian_noise(dd);
    }
    else if (param->noise_type() == 2){
        //return add_ht_cauchy_noise(dd);
    }
    else if (param->noise_type() == 3){
        //return add_ht_gamma_noise(dd);
    }
   // else return 0;
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
    noisy_value = value + dis(gen);

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
    return x + value;
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
    return x + value;
}


