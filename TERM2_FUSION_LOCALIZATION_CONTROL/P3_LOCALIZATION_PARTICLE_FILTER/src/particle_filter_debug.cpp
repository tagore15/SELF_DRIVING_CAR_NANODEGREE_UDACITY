/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;

    std::default_random_engine gen;

    std::normal_distribution<double> N_x(x, std[0]);
    std::normal_distribution<double> N_y(y, std[1]);
    std::normal_distribution<double> N_theta(theta, std[2]);

    for (int i = 0; i < num_particles; i++)
    {
        Particle particle;
        particle.id = i;
        particle.x = N_x(gen);
        particle.y = N_y(gen);
        particle.theta = N_theta(gen);
        particle.weight = 1;

        particles.push_back(particle);
        weights.push_back(1);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    std::default_random_engine gen;

    for (int i = 0; i < num_particles; i++)
    {
        double new_x;
        double new_y;
        double new_theta;

        if (yaw_rate == 0)
        {
            new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
            new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
            new_theta = particles[i].theta;
        }
        else
        {
            new_x = particles[i].x + velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
            new_y = particles[i].y + velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
            new_theta = particles[i].theta + yaw_rate*delta_t;
        }
        std::normal_distribution<double> N_x(new_x, std_pos[0]);
        std::normal_distribution<double> N_y(new_y, std_pos[1]);
        std::normal_distribution<double> N_theta(new_theta, std_pos[2]);

        particles[i].x = N_x(gen);
        particles[i].y = N_y(gen);
        particles[i].theta = N_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
        
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
#if 0
    for (int i = 0; i < map_landmarks.landmark_list.size(); i++)
    {
        cout << map_landmarks.landmark_list[i].x_f << " " << map_landmarks.landmark_list[i].y_f << endl;
    }
    cout << "number of landmarks: " << map_landmarks.landmark_list.size() << endl;

    cout << "number of observations: " << observations.size() << endl;
    for (int i = 0 ; i < observations.size(); i++)
    {
        cout << observations[i].id << " " << observations[i].x << " " << observations[i].y << endl;
    }
    cout << std::endl;
#endif
   //O (2,2) (3,-2) (0,-4)
   //L (5,3) (2,1) (6,1) (7,4) (4,7)  

   // Answers: (6,3), (2,2) and (0,5)

#if 0
   Particle P;
   P.x = 4;
   P.y = 5;
   P.theta = -3.14/2;

   std::vector<LandmarkObs> obs1;
   
   LandmarkObs lt;
   lt.x = 2;
   lt.y = 2;
   obs1.push_back(lt);

   lt.x = 3;
   lt.y = -2;
   obs1.push_back(lt);

   lt.x = 0;
   lt.y = -4;
   obs1.push_back(lt);

   Map mp;
   Map::single_landmark_s single_landmark_temp;
   single_landmark_temp.x_f = 5;
   single_landmark_temp.y_f = 3;
   mp.landmark_list.push_back(single_landmark_temp);

   single_landmark_temp.x_f = 2;
   single_landmark_temp.y_f = 1;
   mp.landmark_list.push_back(single_landmark_temp);

   single_landmark_temp.x_f = 6;
   single_landmark_temp.y_f = 1;
   mp.landmark_list.push_back(single_landmark_temp);

   single_landmark_temp.x_f = 7;
   single_landmark_temp.y_f = 4;
   mp.landmark_list.push_back(single_landmark_temp);
   
   single_landmark_temp.x_f = 4;
   single_landmark_temp.y_f = 7;
   mp.landmark_list.push_back(single_landmark_temp);

   cout << P.x + obs1[0].x * cos(P.theta) - obs1[0].y * sin(P.theta) << endl;
   cout << P.y + obs1[0].x * sin(P.theta) + obs1[0].y * cos(P.theta) << endl;
   cout << P.x + obs1[1].x * cos(P.theta) - obs1[1].y * sin(P.theta) << endl;
   cout << P.y + obs1[1].x * sin(P.theta) + obs1[1].y * cos(P.theta) << endl;
   cout << P.x + obs1[2].x * cos(P.theta) - obs1[2].y * sin(P.theta) << endl;
   cout << P.y + obs1[2].x * sin(P.theta) + obs1[2].y * cos(P.theta) << endl;
   
   std::vector<LandmarkObs> mapObs;
   for (int i = 0; i < obs1.size(); i++)
   {
        lt.x = P.x + obs1[i].x * cos(P.theta) - obs1[i].y * sin(P.theta);
        lt.y = P.y + obs1[i].x * sin(P.theta) + obs1[i].y * cos(P.theta);
        mapObs.push_back(lt);
   }

   for (int i = 0; i < mapObs.size(); i++)
   {
       double min_dist = 123456;
       int L_associated = -1;
       for (int l = 0; l < mp.landmark_list.size(); l++) 
       {
           double dx = (mp.landmark_list[l].x_f - mapObs[i].x);
           double dy = (mp.landmark_list[l].y_f - mapObs[i].y);
           double distance = dx*dx + dy*dy;
           cout << "DISTANCE: " << distance << endl;

           if (min_dist > distance)
           {
             L_associated = l;
             min_dist = distance;
           }
       }
       cout << "L-associated: " << L_associated << endl;

       double diffx = mp.landmark_list[L_associated].x_f - mapObs[i].x;
       double diffy = mp.landmark_list[L_associated].y_f - mapObs[i].y;
       double expo  = ((diffx*diffx)/(2*std_landmark[0]*std_landmark[0])) + ((diffy*diffy)/(2*std_landmark[1]*std_landmark[1]));
       double weight = (1/(2*3.14*std_landmark[0]*std_landmark[1])) * exp(-1 * expo);
       cout << "weight: " << weight << endl;
   }
#endif
   for (int p = 0; p < num_particles; p++)
   {
       particles[p].weight = 1;
       weights[p] = 1;
       std::vector<LandmarkObs> mapObs;
       for (int i = 0; i < observations.size(); i++)
       {
           LandmarkObs lt;
           lt.x = particles[p].x + observations[i].x * cos(particles[p].theta) - observations[i].y * sin(particles[p].theta);
           lt.y = particles[p].y + observations[i].x * sin(particles[p].theta) + observations[i].y * cos(particles[p].theta);
           mapObs.push_back(lt);
       }
       for (int i = 0; i < mapObs.size(); i++)
       {
           double min_dist = 123456;
           int L_associated = -1;
           for (int l = 0; l < map_landmarks.landmark_list.size(); l++) 
           {
               double dx = (map_landmarks.landmark_list[l].x_f - mapObs[i].x);
               double dy = (map_landmarks.landmark_list[l].y_f - mapObs[i].y);
               double distance = dx*dx + dy*dy;

               if (min_dist > distance)
               {
                   L_associated = l;
                   min_dist = distance;
               }
           }
           double diffx = map_landmarks.landmark_list[L_associated].x_f - mapObs[i].x;
           double diffy = map_landmarks.landmark_list[L_associated].y_f - mapObs[i].y;
           double expo  = ((diffx*diffx)/(2*std_landmark[0]*std_landmark[0])) + ((diffy*diffy)/(2*std_landmark[1]*std_landmark[1]));
           particles[p].weight *= ((1/(2*3.14*std_landmark[0]*std_landmark[1])) * exp(-1 * expo));
           weights[p] *= ((1/(2*3.14*std_landmark[0]*std_landmark[1])) * exp(-1 * expo));
       }
   }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> resampledParticles;

     
    for (int i = 0; i < num_particles; i++)
    {
        Particle p;
        int idx = d(gen);
        p.x = particles[idx].x;
        p.y = particles[idx].y;
        p.theta = particles[idx].theta;
        resampledParticles.push_back(p);
    }
    particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
