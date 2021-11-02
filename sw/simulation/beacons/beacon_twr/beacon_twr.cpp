#include "beacon_twr.h"
#include "agent.h"
#include "main.h"
#include "txtwrite.h"
#include <boost/math/special_functions/gamma.hpp>
#include <random>
#define EPSILON 0.001
using namespace std;

beacon_twr::beacon_twr() {
beacon_alg = "beacon_twr";
next_measurement_time = simtime_seconds;

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

float beacon_twr::returnUWBdata(const uint16_t ID, float beacon){
    // function that returns the latest distance entry of uwb data

    // some examples for later implementation
mtx_bcn.lock();
  //  float x_0, y_0; //get coordinates of a beacon
   // float i = 0;//for example beacon 1
   float dist = UWB[ID].back()[0];
   //x_0 = environment.uwb_beacon[i][0];
   //y_0 = environment.uwb_beacon[i][1]; 

    //get the first range measurement for the agent with ID to beacon i
   // UWB[ID][i][1];
mtx_bcn.unlock();
return dist;
    //get the last updated range measurements for the agent to beacon 1
    //cout<<"agent "<<ID<<" distance of "<<en<<" to beacon 1 "<< " (x:"<< environment.uwb_beacon[0][0]<<", y:"<<environment.uwb_beacon[0][1]<<") at timestamp "<<UWB[ID][0].back()[1]<< endl;
    //cout<<"agent "<<ID<<" distance of "<<UWB[ID][1].back()[0]<<" to beacon 2 "<< " (x:"<< environment.uwb_beacon[1][0]<<", y:"<<environment.uwb_beacon[1][1]<<") at timestamp "<<UWB[ID][1].back()[1]<< endl;
   // cout<<"agent "<<ID<<" distance of "<<UWB[ID][2].back()[0]<<" to beacon 3 "<< " (x:"<< environment.uwb_beacon[2][0]<<", y:"<<environment.uwb_beacon[2][1]<<") at timestamp "<<UWB[ID][2].back()[1]<< endl;
    //cout<<"agent "<<ID<<" distance of "<<UWB[ID][3].back()[0]<<" to beacon 4 "<< " (x:"<< environment.uwb_beacon[3][0]<<", y:"<<environment.uwb_beacon[3][1]<<") at timestamp "<<UWB[ID][3].back()[1]<< endl;
}


void beacon_twr::measurement(const uint16_t ID){
    float x_0, y_0, dx0, dy0, d;

    random_device rd;  // Will be used to obtain a seed for the random number engine
    mt19937 rng(rd()); // random-number engine used (Mersenne-Twister in this case)
    uniform_real_distribution<> dis(0.1*param->UWB_interval(), 2*0.1*param->UWB_interval());
    bool static_beacon = false;
    bool dynamic_beacon = false;
    
    //thread safe vector operation to access UWB data
    mtx_bcn.lock();
    uniform_int_distribution<int> uni(0,7+dynamic_uwb_beacon.size()); // guaranteed unbiased
    mtx_bcn.unlock();

    //only take measurements on defined interval with noise
    if(simtime_seconds>=next_measurement_time){
    
    //take measurements from a random beacon that is enabled
    int random_beacon = uni(rng);
    bool beacon_selected = false; 

    // check whether the random beacon is enabled, and whether dynamic or not (if using dynamic beacons is enabled)
    while(beacon_selected == false){
        if(random_beacon >= 0 && random_beacon <=7){
            if(environment.uwb_beacon[random_beacon][2]==0){
            random_beacon= uni(rng);
            }else if(environment.uwb_beacon[random_beacon][2]==1){
            beacon_selected = true;
            static_beacon = true;
            }
        }
        
        if(random_beacon>7){
            mtx_bcn.lock();
            random_beacon = random_beacon-8;
            if(dynamic_uwb_beacon[random_beacon][2]==0){
            random_beacon= uni(rng);
            }else if(dynamic_uwb_beacon[random_beacon][2]==1 && random_beacon == ID){
                if(s.size() == 1 ){
                    beacon_selected = true;
                    static_beacon = true;
                    random_beacon = 1;
                }
                else{
                    random_beacon= uni(rng);
                }
            
            }else if(dynamic_uwb_beacon[random_beacon][2]==1 && random_beacon != ID){
            beacon_selected = true;
            dynamic_beacon = true;
            std::cout<<"agent "<<ID<<" ranges with agent "<<random_beacon<<std::endl;
            }
           mtx_bcn.unlock(); 
        }
    }

    // coordinates of the beacon
    if(dynamic_beacon == true){
        mtx_bcn.lock();
    x_0 = dynamic_uwb_beacon[random_beacon][0];
    y_0 = dynamic_uwb_beacon[random_beacon][1]; 
    mtx_bcn.unlock();
    }
    if(static_beacon == true){
    x_0 = environment.uwb_beacon[random_beacon][0];
    y_0 = environment.uwb_beacon[random_beacon][1];
    }
     
    //generate twr measurements
    dx0 = s[ID]->state[0] - x_0;
    dy0 = s[ID]->state[1] - y_0;
    d = sqrt(dx0*dx0 + dy0*dy0);

    //initialise uwb dataset
    mtx_bcn.lock();
    UWB.push_back(std::vector<std::vector<float>>());
    UWB[ID].push_back(std::vector<float>());

    //add preferred noise on top of twr measurements
    if (param->noise_type() == 0){
        // twr measurement, x beacon, y beacon, simulation time
        UWB[ID].push_back({d,x_0, y_0, simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 1){
        // twr measurement, x beacon, y beacon, simulation time
        UWB[ID].push_back({add_gaussian_noise(d),x_0, y_0, simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 2){
        // twr measurement, x beacon, y beacon, simulation time
        UWB[ID].push_back({add_ht_cauchy_noise(d),x_0, y_0, simtime_seconds});
        mtx_bcn.unlock();
    }
    else if (param->noise_type() == 3){
        // twr measurement, x beacon, y beacon, simulation time
        UWB[ID].push_back({add_ht_gamma_noise(d),x_0, y_0, simtime_seconds});
        mtx_bcn.unlock();
    }
    }
    //next measurement time interval
    next_measurement_time = next_measurement_time + param->UWB_interval() + dis(rng) -0.1*param->UWB_interval();
    std::cout<<"dynamic beacon"<<dynamic_beacon<<"static beacon"<<static_beacon<<std::endl;

}

float beacon_twr::add_gaussian_noise(float value) {
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

float beacon_twr::add_ht_cauchy_noise(float value){
    float cauchy_alpha = (2*M_PI*param->htc_gamma()) / (sqrt(2*M_PI*pow(param->gauss_sigma(),2)) + M_PI*param->htc_gamma());
    float CDF_limit = (2-cauchy_alpha)*0.5;
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (tmp < CDF_limit){
        //Gaussian CDF: F(x) = (1/2) * (1 + erf( (x-mu)/sqrt(2*sig^2) ))
        return value + value*sqrt(2*pow(param->gauss_sigma(),2))*erfinvf(2*tmp/(2-cauchy_alpha)-1);
    }
    else{
        //Cauchy CDF: F(x) = (1/pi) * arctan((x-x0)/gamma) + 1/2
        //Because of scaling with alpha (Area under CDF no longer 1), need to also start from -inf and then flip
        tmp = 1-tmp;
        return value -value*param->htc_gamma()*tan(M_PI*(tmp/cauchy_alpha-0.5));
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
    return value*x + value;
}

