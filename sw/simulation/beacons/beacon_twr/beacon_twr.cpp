#include "beacon_twr.h"
#include "agent.h"
#include "main.h"
#include "txtwrite.h"
#include <boost/math/special_functions/gamma.hpp>
#define EPSILON 0.001
using namespace std;
beacon_twr::beacon_twr() {

}

void beacon_twr::ranges_terminal(const uint16_t ID){
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

void beacon_twr::measurement2pos(const uint16_t ID){
    measurement(ID);

    // some examples for later implementation

    float x_0, y_0; //get coordinates of a beacon
    float i = 0;//for example beacon 1

    x_0 = environment.uwb_beacon[i][0];
    y_0 = environment.uwb_beacon[i][1]; 

    //get the first range measurement for the agent with ID to beacon i
    UWB[ID][i][1];


    //get the last updated range measurements for the agent to beacon 1,2,3 and 4
    cout<<"distance to beacon 1 "<<UWB[ID][0].back()<< endl;
    cout<<"distance to beacon 2 "<<UWB[ID][1].back()<< endl;
    cout<<"distance to beacon 3 "<<UWB[ID][2].back()<< endl;
    cout<<"distance to beacon 4 "<<UWB[ID][3].back()<< endl;
}


void beacon_twr::measurement(const uint16_t ID){
    float x_0, y_0, dx0, dy0, d;
    
    for (size_t i = 0; i < 8; i++){
        UWB.push_back(std::vector<std::vector<float>>());
        UWB[ID].push_back(std::vector<float>());
        //UWB[ID][i].push_back(environment.uwb_beacon[i][0]);
        //UWB[ID][i].push_back(environment.uwb_beacon[i][1]);
        //UWB[ID][i].push_back(0);
        //UWB[ID][i].push_back(std::vector<float>());
        x_0 = environment.uwb_beacon[i][0];
        y_0 = environment.uwb_beacon[i][1]; 

        //x_0 = UWB[ID][i][0];
        //y_0 = UWB[ID][i][1];

        dx0 = s[ID]->get_position(0) - x_0;
        dy0 = s[ID]->get_position(1) - y_0;
        d = sqrt(dx0*dx0 + dy0*dy0);
        if (param->noise_type() == 0){
          UWB[ID][i].push_back(d);
         // environment.uwb_beacon[i].push_back(ID);
          
        }
        else if (param->noise_type() == 1){
            //environment.uwb_beacon[i].push_back(add_gaussian_noise(d));
            UWB[ID][i].push_back(add_gaussian_noise(d));
        
        }
        else if (param->noise_type() == 2){
            //environment.uwb_beacon[i].push_back(add_ht_cauchy_noise(d));
            UWB[ID][i].push_back(add_ht_cauchy_noise(d));
            
        }
        else if (param->noise_type() == 3){
           // environment.uwb_beacon[i].push_back(add_ht_gamma_noise(d));
            UWB[ID][i].push_back(add_ht_gamma_noise(d));
            
        }
        
        
    }
   
}

float beacon_twr::add_gaussian_noise(float value) {
    float noisy_value;
    float mean = 0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, param->gauss_sigma());
    // Add Gaussian noise
    noisy_value = value + dist(generator);
    return noisy_value;
}

float beacon_twr::add_ht_cauchy_noise(float value){
    float cauchy_alpha = (2*M_PI*param->htc_gamma()) / (sqrt(2*M_PI*pow(param->gauss_sigma(),2)) + M_PI*param->htc_gamma());
    float CDF_limit = (2-cauchy_alpha)*0.5;
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (tmp < CDF_limit){
        //Gaussian CDF: F(x) = (1/2) * (1 + erf( (x-mu)/sqrt(2*sig^2) ))
        return value + sqrt(2*pow(param->gauss_sigma(),2))*erfinvf(2*tmp/(2-cauchy_alpha)-1);
    }
    else{
        //Cauchy CDF: F(x) = (1/pi) * arctan((x-x0)/gamma) + 1/2
        //Because of scaling with alpha (Area under CDF no longer 1), need to also start from -inf and then flip
        tmp = 1-tmp;
        return value -param->htc_gamma()*tan(M_PI*(tmp/cauchy_alpha-0.5));
    }
}

//our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_twr::cdf_ht_gamma(double x)
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
double beacon_twr::deriv_cdf_ht_gamma(double x)
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
float beacon_twr::add_ht_gamma_noise(float value){
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

