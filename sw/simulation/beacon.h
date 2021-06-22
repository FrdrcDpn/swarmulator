#ifndef BEACON_H
#define BEACON_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

/**
 * Parent class defining a beacon. The dynamic implementation is handled in children classes.
 */

class Beacon
        {
public:
    /**
     * Constructor
     */
    Beacon();
    std::vector<float> range_beacon(const uint16_t ID);
    /**
     * Destructor
     */
    virtual ~Beacon() {};

    /**
   * @brief Return noisy value following gaussian distribution
   *
   * @param value value to which noise is to be added
   * @param mean mean of the gaussian noise
   * @param stddev standard deviation of the gaussian noise
   */
    float add_gaussian_noise(float value, const double mean, const double stddev);

    /**
    * @brief Return noisy value following cauchy distribution
    *
    * @param value value to which noise is to be added
    * @param mean mean of the gaussian noise
    * @param stddev standard deviation of the gaussian noise
    */
    float add_ht_cauchy_noise(float value, const double mean, const double stddev);


    /**
    * @brief Output exact distances from agent to all active beacons to the terminal
    *
    * @param ID agent
    */
    virtual void ranges_terminal(const uint16_t ID)=0;

    /**
    * @brief Output twr range from agent to all active beacons to the terminal
    *
    * @param ID agent
    */
    virtual float range(const uint16_t ID,const uint16_t ID_beacon )=0;

};


#endif //BEACON_H
