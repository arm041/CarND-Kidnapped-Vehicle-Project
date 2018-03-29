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

	cout << "Start of initialization!!!!"<< endl;

	num_particles = 200; //can be changed later to another number by arm041

	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta


	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// This line creates a normal (Gaussian) distribution for x, y, and theta.
	normal_distribution<double> dist_x(0.0, std_x);
	normal_distribution <double> dist_y(0.0,std_y);
	normal_distribution <double> dist_theta (0.0, std_theta);

	for (int i = 0; i < num_particles ; i++)
	{
		double noise_x = dist_x (gen);
		double noise_y = dist_y(gen);
		double noise_theta = dist_theta (gen);

		Particle temp;

		temp.id = i;
		temp.x = x + noise_x;
		temp.y = y + noise_y;
		temp.theta = theta + noise_theta;

		//normalize the angles
		/*while (temp.theta < -M_PI)
			temp.theta += 2 * M_PI;
		while (temp.theta > M_PI)
			temp.theta -= 2 * M_PI;*/

		temp.weight = 1.0;
		particles.push_back(temp);
		weights.push_back (1.0);
	}


	is_initialized = true;

	cout << "initialization successful!!!!" << endl;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	cout << "start of prediction!!!"<< endl;

	default_random_engine gen;
	double std_x, std_y, std_yaw; // Standard deviations for x, y, and theta


	std_x = std_pos[0];
	std_y = std_pos[1];
	std_yaw = std_pos[2];

	cout << "start of prediction!!!"<< endl;
	normal_distribution<double> dist_x(0.0, std_x);
	normal_distribution <double> dist_y(0.0,std_y);
	normal_distribution <double> dist_yaw (0.0, std_yaw);

	for (int i = 0; i < num_particles ; i++)
	{

		// This line creates a normal (Gaussian) distribution for x, y, and theta.


		double pos_x = dist_x (gen);
		double pos_y = dist_y(gen);
		double pos_yaw = dist_yaw (gen);



		if (yaw_rate < 0.00000000001)
		{
			particles[i].x +=  (velocity*cos(particles[i].theta) * delta_t + pos_x);
			particles[i].y += (velocity*sin(particles[i].theta) * delta_t + pos_y);
			particles[i].theta += pos_yaw;
		}
		else
		{
			double angle = particles[i].theta + yaw_rate * delta_t;


			particles[i].x += (velocity/yaw_rate) * (sin(angle) - sin(particles[i].theta)) + pos_x;
			particles[i].y += (velocity/yaw_rate) * (-1*cos(angle) + cos(particles[i].theta)) + pos_y;
			particles[i].theta += (yaw_rate * delta_t) + pos_yaw;
		}

		//normalize the angles
		/*while (particles[i].theta < -M_PI)
			particles[i].theta += 2 * M_PI;
		while (particles[i].theta > M_PI)
			particles[i].theta -= 2 * M_PI;*/
	}
	cout << "end of prediction!!!"<< endl;

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double nearest_neighbour;
	for (int i = 0; i < observations.size() ; i++)
	{
		nearest_neighbour =  dist (observations[i].x, observations[i].y, predicted[0].x, predicted[0].y);
		for (int j = 1 ; j < predicted.size() ; j++)
		{
			if (nearest_neighbour > dist (predicted[j].x, predicted[j].y, observations[i].x, observations[i].y))
			{
				nearest_neighbour = dist (predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
				observations[i].id = predicted[j].id;
			}
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

	cout << "start of weight update!!!"<< endl;

	double std_x = std_landmark[0];
	double std_y = std_landmark[1];

	double var_x = std_x * std_x;
	double var_y = std_y * std_y;
	std::vector <LandmarkObs> observations_tf ;
	std::vector <LandmarkObs> predicted;
	for (int i = 0; i < num_particles ; i++)
	{
		observations_tf.clear();
		predicted.clear();
		//particles[i].weight = 1.0;

		//transform observations and call them observations_tf by arm041
		std::vector <LandmarkObs> observations_tf ;
		for (int j = 0; j < observations.size(); j++)
		{
			LandmarkObs temp;
			temp.x = particles[i].x + observations[j].x *  cos(particles[i].theta) - observations[j].y *  sin(particles[i].theta);
			temp.y = particles[i].y + observations[j].x *  sin(particles[i].theta) + observations[j].y *  cos(particles[i].theta);
			//temp.id = observations[i].id;

			observations_tf.push_back (temp);
			cout << "observation.x: " << observations[j].x << " observations transformed.x: " << observations_tf[i].x << endl;
			cout << "observation.y: " << observations[j].y << " observations transformed.y: " << observations_tf[i].y << endl;
		}
		//cout << "observation size: " << observations.size() << "and transformed size: " << observations_tf.size() << endl;

		// create predicted vector by going through landmarks in the map and choosing the ones in the range of sensors
		for (int j = 0; j < map_landmarks.landmark_list.size() ; j++)
		{
			//cout << "wow: " << (dist (particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f)) << "sensor ragnge: " << sensor_range << endl;
			if ((dist (particles[i].x, particles[i].y, map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f)) <= sensor_range)
			{
				LandmarkObs temp;
				temp.x = map_landmarks.landmark_list[i].x_f;
				temp.y = map_landmarks.landmark_list[i].y_f;
				temp.id = map_landmarks.landmark_list[i].id_i;
				predicted.push_back(temp);
			}
		}
		cout << "size of predicted vector: " << predicted.size() << endl;
		dataAssociation (predicted, observations_tf);

		particles[i].weight = 1.0;

		//update weights by arm041
		for (int j = 0; j < observations_tf.size(); j++)
		{
			for (int k = 0; k < predicted.size();k++)
			{
				if (observations_tf[j].id == predicted[k].id)
				{
					particles[i].weight *=  (exp(-1*( (pow((observations_tf[j].x - predicted[k].x),2) / (2 * var_x)) +  (pow((observations_tf[j].y - predicted[k].y),2) / (2 * var_y))))) / (2.0 * M_PI * std_x * std_y);
					continue;
				}
			}
		}
		//cout << particles[i].weight<< endl;

		weights[i] = particles[i].weight;
		cout << "In Weight calculation phase Weight" << i << " = " << weights[i] << endl;
	}

	cout << "end of weight update!!!"<< endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	cout << "start of weight resample!!!"<< endl;
	vector<Particle> particles_temp;
    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> d(weights.begin(), weights.end());

    for(int i = 0; i < num_particles; i++) {
    	int test = d(gen);
        particles_temp.push_back(particles[test]);

    }
	particles = particles_temp;


	cout << "end of weight resample!!!"<< endl;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
