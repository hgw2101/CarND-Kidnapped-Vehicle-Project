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

#define PI 3.14159265

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	// is_initialized = false; don't need this since the constructor already has this set as false
	num_particles = 1000; //try to experiment with different numbers
	// default_random_engine gen; //random number generator

	// normal_distribution<double> dist_x(x, std[0]);
	// normal_distribution<double> dist_y(y, std[1]);
	// normal_distribution<double> dist_psi(theta, std[2]);

	for (int i=0; i<num_particles; i++) {
		Particle p = {
			i, //id
			generate_random_gaussian_value(x, std[0]), //value with noise
			generate_random_gaussian_value(y, std[1]), 
			generate_random_gaussian_value(theta, std[2]),
			1 // initial weight
		};

		particles.push_back(p);
		weights.push_back(w);
	}

	is_initialized = true;

	cout<<"this is particles size: "<<particles.size()<<endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// this step predicts the particle's next position, then together with the observations in vehicle coordinates, we 
	// transform these observations into map coordinates and then associate each observation with the nearest landmark

	for (int i=0; i<num_particles; i++) {
		double new_x_mean = particles[i].x;
		double new_y_mean = particles[i].y;
		double new_theta_mean = particles[i].theta;

		// update position and yaw values, this is the same as UKF mean state vector update, but the noise part here is different from the process noise in UKF
		if (fabs(yaw_rate) > 0.001) {
			new_x_mean += velocity/yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta));
			new_y_mean += velocity/yaw_rate * (cos(theta - cos(theta+yaw_rate*delta_t)));
			new_theta_mean += yaw_rate * delta_t;
		} else {
			new_x_mean += velocity * cos(yaw_rate) * delta_t;
			new_y_mean += velocity * sin(yaw_rate) * delta_t;
		}

		particles[i].x = generate_random_gaussian_value(x, std_pos[0]);
		particles[i].y = generate_random_gaussian_value(y, std_pos[1]);
		particles[i].theta = generate_random_gaussian_value(theta, std_pos[2]);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	// both predicted and observations are LandmarkObs objects, i.e. in Map coordinates
	// double loop, iterate through all predicted measurements, and for each predicted measurement, find the closest observation

	/*
	* Struct representing one landmark observation measurement.
	*/
	struct ObsDist {
		int id;				// Id of matching landmark in the map.
		double distance; // Absolute distance between predicted observation and the observed landmark
	};

	//TODO!!
	std::vector<ObsDist> nearest_obs;
	for (int i=0; i<predicted.size(); i++) {
		double predicted_x = predicted[i].x;
		double predicted_y = predicted[i].y;
		for (int j=0; j<observations.size(); j++) {
			double obs_x = observations[j].x;
			double obs_y = observations[j].y;

			//use the dist() helper function 

			// double distance = sqrt(pow(obs_x-predicted_x, 2) + pow(obs_y-predicted_y, 2));

			if (nearest_obs)

			distances.push_back(distance); // TODO: need to find a way to track the id of the closest obs
		}
		// find the smallest distance
		std::sort(distances.begin(), distances.end());

	}


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

	// step 0) set Gaussian terms for easy access and avoid repeated calculations
	double std_x = std_landmark[0];
	double std_y = std_landmark[1];
	double gaussian_norm = 1 / (2*PI*std_x*std_y);
	double x_div = 2*std_x*std_x;
	double y_div = 2*std_y*std_y;

	// transform first, then add noise
	// step 1) transformation
	for (int i=0; i<num_particles; i++) {
		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// step 2) get a list of landmark objects based on the sensor_range
		std::vector<LandmarkObs> landmarks_in_range;
		for (int j=0; j<map_landmarks.landmark_list.size(); j++) {
			Map::single_landmark_s landmark = map_landmarks.landmark_list[j];

			if (dist(landmark.x, landmark.y, x, y) < sensor_range) {
				landmarks_in_range.push_back(landmark);
			}
		}

		//iterate through list of observations
		for (int k=0; k<observations.size(); k++) {
			LandmarkObs obs = observations[k];

			// step 3) convert obs from vehicle coordinates to map coordinates
			double transformed_obs_x_mean = obs.x * cos(theta) - obs.y * sin(theta) + x;
			double transformed_obs_y_mean = obs.y * sin(theta) + obs.y * cos(theta) + y;

			LandmarkObs trans_obs;
			trans_obs.x = generate_random_gaussian_value(transformed_obs_x_mean, std_landmark[0]);
			trans_obs.y = generate_random_gaussian_value(transformed_obs_y_mean, std_landmark[1]);

			// step 4) for each trans_obs, find the landmark association using nearest neighbor
			// TODO: optimize performance, instead of using euclidean distance, use x/y distance
			int min_id;
			double min_dist = 1000.0; // set to max distance
			for (int n=0; n<landmarks_in_range; n++) {
				Map::single_landmark_s curr_landmark = landmarks_in_range[n];
				double dist = dist(curr_landmark.x_f, curr_landmark.y_f, trans_obs.x, trans_obs.y);
				if (dist < min_dist) {
					min_id = curr_landmark.id_i;
					min_dist = dist;
				}
			}

			// step 5) now we have the observation matched with a landmark, calculate the partial weight
			double mean_x = landmarks_in_range[min_id].x_f;
			double mean_y = landmarks_in_range[min_id].y_f;
			double exponent = (pow(trans_obs.x-mean_x,2)/x_div + pow(trans_obs.y-mean_y,2)/y_div) * -1;
			double partial_weight = gaussian_norm * exp(exponent);

			// step 6) finally, update the final weight for this particle
			weights[i] *= partial_weight;
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resampled_particles;

	for (int i=0; i<num_particles; i++) {
		resampled_particles.push_back(particles[distribution(gen)]);
	}

	particles = resampled_particles;
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
