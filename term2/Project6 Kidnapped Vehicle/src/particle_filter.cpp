/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

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
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 1000;  // TODO: Set the number of particles
  
  // Random number generator
  default_random_engine gen;
  
  // Normal distributions for initial coordinates
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  particles.resize(num_particles);
  weights.resize(num_particles);
  
  double weight=1.0;
  
  for (int i=0; i<num_particles; ++i)
  {
    double position_x=dist_x(gen);
    double position_y=dist_y(gen);
    double position_theta=dist_theta(gen);
    
    particles[i]={i, position_x, position_y, position_theta, weight};
    weights[i]=weight;
  }
  
  is_initialized=true;
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
    
  default_random_engine gen;
  
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);
  
  //prediction
  for (int i=0; i<num_particles; ++i)
  {
   if (fabs(yaw_rate)>0.01)
   {
     double x_p=particles[i].x;
     double y_p=particles[i].y;
     double theta_p=particles[i].theta;
   
     particles[i].x=x_p+(velocity/yaw_rate)*(sin(theta_p+yaw_rate*delta_t)-sin(theta_p));
     particles[i].y=y_p+(velocity/yaw_rate)*(cos(theta_p)-cos(theta_p+yaw_rate*delta_t));
     particles[i].theta=theta_p+yaw_rate*delta_t;
   }
   else 
   {
     double x_p=particles[i].x;
     double y_p=particles[i].y;
     double theta_p=particles[i].theta;
     
     particles[i].x=x_p+velocity*delta_t*cos(theta_p);
     particles[i].y=y_p+velocity*delta_t*sin(theta_p);
    
   }
  particles[i].x+=dist_x(gen);
  particles[i].y+=dist_y(gen);
  particles[i].theta+=dist_theta(gen);
  
  
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	for (LandmarkObs & obs : observations)
    {
    	double min_distance=numeric_limits<double>::max();
        int min_id;
        for(const LandmarkObs&pred:predicted)
        {
        	const double distance=dist(obs.x, obs.y, pred.x, pred.y);
            if(distance<min_distance)
            {
            	min_distance=distance;
                min_id=pred.id;
            }
        
        }
    	obs.id=min_id;
    
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for (int i=0; i<num_particles; ++i)
  {
  	double id_up=particles[i].id;
    double x_up=particles[i].x;
    double y_up=particles[i].y;
  	double theta_up=particles[i].theta;
    double weight_up=particles[i].weight;
    
    vector<LandmarkObs>obs_trans;
    
    for(int j=0; j<observations.size(); ++j)
    {
    	int obs_id=observations[j].id;
      	double obs_x=observations[j].x;
      	double obs_y=observations[j].y;
      
      	double trans_obs_x=x_up+cos(theta_up)*obs_x-sin(theta_up)*obs_y;
      	double trans_obs_y=y_up+sin(theta_up)*obs_x+cos(theta_up)*obs_y;
    
    	obs_trans.push_back(LandmarkObs{obs_id, trans_obs_x, trans_obs_y});
    }
    
    vector<LandmarkObs>predicted;
    
    for(int j=0; j<map_landmarks.landmark_list.size(); ++j)
    {
    	int land_id=map_landmarks.landmark_list[j].id_i;
      	double land_x=map_landmarks.landmark_list[j].x_f;
      	double land_y=map_landmarks.landmark_list[j].y_f;
      
      	if(sqrt((x_up-land_x)*(x_up-land_x)+(y_up-land_y)*(y_up-land_y))<sensor_range)
        {
        	predicted.push_back(LandmarkObs{land_id, land_x, land_y});
                
        }
     
    }
    
    dataAssociation(predicted, obs_trans);
    
    particles[i].weight=1;
    
    for(int j=0; j<obs_trans.size(); ++j)
    {
    	int trans_obs_id=obs_trans[j].id;
      	double trans_obs_x=obs_trans[j].x;
      	double trans_obs_y=obs_trans[j].y;
      
      
      	double predicted_x;
      	double predicted_y;
      
      	for(int k=0; k<predicted.size(); ++k)
        {
        	if(trans_obs_id==predicted[k].id)
            {
            	predicted_x=predicted[k].x;
              	predicted_y=predicted[k].y;
            }
               
        }
    
    	double sigma_x=std_landmark[0];
      	double sigma_y=std_landmark[1];
      
      	double x2=(trans_obs_x-predicted_x)*(trans_obs_x-predicted_x);
      	double y2=(trans_obs_y-predicted_y)*(trans_obs_y-predicted_y);
      	double sigma_x2=sigma_x*sigma_x;
      	double sigma_y2=sigma_y*sigma_y;
      
      	float p_up=(1/(2*M_PI*sigma_x*sigma_y))*exp(-((x2/(2*sigma_x2))+(y2/(2*sigma_y2))));
      
      	particles[i].weight*=p_up;
    
    }
        
  }

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
 	default_random_engine gen;
  	
  	uniform_real_distribution<double> distribution(0.0,1.0);
  
  	double beta=0.0;
  
  	double w_max=0;
  
  	for(int i=0; i<num_particles; ++i)
    {
      	if(particles[i].weight>w_max)
        {
        	w_max=particles[i].weight;
        }
    	
    }
  
  	int index=int(distribution(gen)*num_particles);
  
  	for(int i=0; i<num_particles; ++i)
    {
    	beta=beta+distribution(gen)*2.0*w_max;
      	while(particles[index].weight<beta)
        {
        	beta=beta-particles[index].weight;
          	index=(index+1)%num_particles;
             
        }
      	particles[i]=particles[index];
    }  
  
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}