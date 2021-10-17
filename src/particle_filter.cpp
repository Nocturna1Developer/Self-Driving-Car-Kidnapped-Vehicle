/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

// ALL CREDIT: Implementing a particle filter lesson, https://github.com/felipemartinezs/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp (used as a reference), gaussian sampling code solution, motion model lesson, https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm, http://planning.cs.uiuc.edu/node99.html

#include "particle_filter.h"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include "helper_functions.h"

using std::string;
using std::vector;

// CREDIT: Program Gaussian Sampling: Code Solution
void ParticleFilter::init(double x, double y, double theta, double std[])
{
    // TODO: Set the number of particles
    num_particles = 100;

    // Variables to be used in the for loop
    std::default_random_engine gen;

    // This line creates a normal (Gaussian) distribution for x, y, and theta - Guassian sampling quiz
    // TODO: Add random Gaussian noise to each particle.
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++)
    {
        // TODO: Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.
        Particle myParticle;
        myParticle.weight = 1.;
        myParticle.id = i;
        // TODO: Sample from these normal distributions like this:
        // 		sample_x = dist_x(gen);
        // 		where "gen" is the random engine initialized earlier. - Guassian sampling quiz
        myParticle.x = dist_x(gen);
        myParticle.y = dist_y(gen);
        myParticle.theta = dist_theta(gen);

        // Print your samples to the terminal.
        //std::cout << "Sample " << i + 1 << " " << myParticle.x << " " << myParticle.y << " "
        //<< myParticle.theta << std::endl;

        particles.push_back(myParticle);
    }

    is_initialized = true;
}

// CREDIT: Motion models lesson, Program Gaussian Sampling: Code Solution, https://www.tutorialspoint.com/fabs-in-cplusplus
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate)
{
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    std::default_random_engine gen;
    // This line creates a normal (Gaussian) distribution for x, y, and theta - Guassian sampling quiz
    // TODO: Add random Gaussian noise to each particle.
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);

    // TODO: Add measurements to each particle and add random Gaussian noise.
    for (int i = 0; i < num_particles; i++)
    {
        if (fabs(yaw_rate) < 0.00001)
        {
            // According to the Yaw rate and velocity video under modtion models:
            // x_f = x_0 + v(dt)*cos(theta_0)
            // y_f = y_0 + v(dt)*sin(theta_0)
            // x_f, y_f = final x and y positions
            // x_0, y_0 = initial y position
            // v = velocity
            // dt = time elapsed
            // cos, sin = x and y component
            // Utilizing information about the video we are able to do the following:
            // NOte: particles - set of current particles according to particle filter.h
            particles[i].x += velocity * delta_t * cos(particles[i].theta);
            particles[i].y += velocity * delta_t * sin(particles[i].theta);
            //particles[i].theta += velocity * delta_t * sin(particles[i].theta);
        }
        else
        {
            // According to the Yaw rate and velocity video under modtion models:
            // x_f = x_0 + v/yaw_rate[yaw_rate_0 + yaw_rate(dt) - sin(yaw_rate_0)]
            // y_f = y_0 + v/yaw_rate[yaw_rate_0 + yaw_rate(dt) + yaw_rate(dt)]
            // final_yaw = inital_yaw + yaw_rate(dt)
            // x_f, y_f = final x and y positions
            // yaw_rate_0 = initial yaw rate
            // v = velocity
            // dt = time elapsed
            // cos, sin = x and y component
            // Utilizing information about the video we are able to do the following:
            particles[i].x += velocity / yaw_rate * (-sin(particles[i].theta) + sin(particles[i].theta + yaw_rate * delta_t));
            particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            particles[i].theta += yaw_rate * delta_t;
        }
        // The following will add noise to filter
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

// CREDIT : Implementation of a particle filter, landmarks quiz, converting land mark observations lesson, https://www.learncpp.com/cpp-tutorial/unsigned-integers-and-why-to-avoid-them/#:~:text=C%2B%2B%20also%20supports%20unsigned%20integers,hold%20non%2Dnegative%20whole%20numbers.&text=Both%20can%20store%20256%20different,that%20are%20twice%20as%20large.
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs> &observations)
{
    // Declaring the nessesary variables
    double minimum_distance, current_distance;
    //double minimum_distance, current_distance, x_value_O, y_value_O, x_value_P, y_value_P;
    int landmark_observation_id;

    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.

    //x_value_O = observations[i].x;
    //y_value_O = observations[i].y;
    //x_value_P = predicted[j].x;
    //y_value_P = predicted[j].y;

    for (unsigned int i = 0; i < observations.size(); i++)
    {
        // Minimum to maximum
        minimum_distance = std::numeric_limits<double>::max();

        for (unsigned j = 0; j < predicted.size(); j++)
        {
            // Figuring out the distance of the observed and predicted states for x and y thanks to the double for loop
            current_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

            // Getting the nearest landmark
            if (current_distance < minimum_distance)
            {
                minimum_distance = current_distance;
                landmark_observation_id = predicted[j].id;
            }

            observations[i].id = landmark_observation_id;
        }
    }
}

// CREDIT: Implementation of a Particle Filter lesson, https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm, http://planning.cs.uiuc.edu/node99.html, https://en.wikipedia.org/wiki/Multivariate_normal_distribution
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], const vector<LandmarkObs> &observations, const Map &map_landmarks)
{
    // Your particles are located according to the MAP'S coordinate system. You will need to transform between the two systems. This transformation requires both rotation AND translation

    // Declaring needed variables to the particle updates
    double x_par, y_par, theta_par;

    // Declaring needed variables to the landmark updates, and the vector which will store/hold predticion values
    double x_land, y_land;
    int id_land;

    // Declaring needed variables for the observation updates and a vector that keeps track of the observed corrdinates
    double x_obs, y_obs;
    int id_obs;

    // Declaring needed place holder variables in which each weight will be calculated using the Multivariate gaussian distribution
    double observation_x_up, observation_y_up;
    double prediction_x_up, prediction_y_up;

    // Declaring needed variables to normalize values
    // double x_sigma, y_sigma;
    double normalized;

    // Update the weights of each particle using a mult-variate Gaussian distribution
    for (int i = 0; i < num_particles; i++)
    {
        // Coordinates of x,y,theta
        x_par = particles[i].x;
        y_par = particles[i].y;
        theta_par = particles[i].theta;

        // Landmarks
        vector<LandmarkObs> predictions_vector;
        for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++)
        {
            x_land = map_landmarks.landmark_list[j].x_f;
            y_land = map_landmarks.landmark_list[j].y_f;

            id_land = map_landmarks.landmark_list[j].id_i;

            // Predict measurements of land mark within sesor range - Explanation of project code video
            if (fabs(x_land - x_par) <= sensor_range && fabs(y_land - y_par) <= sensor_range)
            {
                predictions_vector.push_back(LandmarkObs{id_land, x_land, y_land});
            }
        } // END OF LOOP j

        // Observations
        vector<LandmarkObs> observations_vector;
        for (unsigned int k = 0; k < observations.size(); k++)
        {
            x_obs = x_par + cos(theta_par) * observations[k].x - sin(theta_par) * observations[k].y;
            y_obs = y_par + sin(theta_par) * observations[k].x + cos(theta_par) * observations[k].y;
            id_obs = observations[k].id;

            observations_vector.push_back(LandmarkObs{id_obs, x_obs, y_obs});
        } // END OF LOOP k

        // "Once we have the predicted land mark measurements we can use this function to associate sensor measurements to the landmarks"
        dataAssociation(predictions_vector, observations_vector);

        // Makes sure the particle weight is one
        particles[i].weight = 1.0;

        // "We will need these to calculate the new weight of each particle using multivaritate guaaisan distribution" - Explanation of project code video
        for (unsigned int m = 0; m < observations_vector.size(); m++)
        {
            observation_x_up = observations_vector[m].x;
            observation_y_up = observations_vector[m].y;

            for (unsigned int r = 0; r < predictions_vector.size(); r++)
            {
                if (predictions_vector[r].id == observations_vector[m].id)
                {
                    prediction_x_up = predictions_vector[r].x;
                    prediction_y_up = predictions_vector[r].y;
                }
            } // END OF LOOP r

            // Now we need to calculate these particle wights and the we need to normalized these wieghts
            // This formula was taken from the Particle weight solution lesson
            // P(x,y) = 1/(2*pi*sigma_x*sigma_y) * e^(-x - mu_x)^2)/2*sigma^2_x + (y - mu_y)^2/2sigma^2_y)
            normalized = (1. / (2. * 3.14 * std_landmark[0] * std_landmark[1])) * exp(-(pow((observation_x_up - prediction_x_up), 2) / (2 * pow(std_landmark[0], 2)) + (pow((observation_y_up - prediction_y_up), 2) / (2 * pow(std_landmark[1], 2)))));

            particles[i].weight *= normalized;
        } // END OF LOOP m
    }     // END OF LOOP i
}

// Credit: http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution, project video explanation video code, https://github.com/felipemartinezs/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp (used as reference)
void ParticleFilter::resample()
{
    // TODO: Resample particles with replacement with probability proportional.

    // We are once again to get random particle generations
    std::default_random_engine gen;

    // Vector that gets the weight of current particles
    vector<double> particle_weights;

    // This will get the max weight and distibute it from 0 to the max weight
    double maximum = std::numeric_limits<double>::min();

    for (int i = 0; i < num_particles; i++)
    {
        particle_weights.push_back(particles[i].weight);
        // Prevents error from being larger than the max
        if (particles[i].weight > maximum)
        {
            maximum = particles[i].weight;
        }
    }

    // Randomly uniformly distibutes the weight
    std::uniform_real_distribution<double> dist_double(0.0, maximum);
    std::uniform_int_distribution<int> dist_int(0, num_particles - 1);

    // This determines a random starting index
    int rand_Index = dist_int(gen);
    double beta_value = 0.;

    // Vector that gets the rersampled particles
    vector<Particle> particle_resample;

    for (int j = 0; j < num_particles; j++)
    {
        beta_value += dist_double(gen) * 2.;
        while (beta_value > particle_weights[rand_Index])
        {
            beta_value -= particle_weights[rand_Index];
            rand_Index = (rand_Index + 1) % num_particles;
        }
        particle_resample.push_back(particles[rand_Index]);
    }
    particles = particle_resample;
}

void ParticleFilter::SetAssociations(Particle &particle, const vector<int> &associations, const vector<double> &sense_x, const vector<double> &sense_y)
{
    // particle: the particle to which assign each listed association,
    //   and association's (x,y) world coordinates mapping
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
    vector<double> v;

    if (coord == "X")
    {
        v = best.sense_x;
    }
    else
    {
        v = best.sense_y;
    }

    std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1); // get rid of the trailing space
    return s;
}