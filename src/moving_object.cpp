/*********************************************************************************************************************
 * File : moving_object.cpp                                                                                          *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 *********************************************************************************************************************/

#include "moving_object.h"

// Initialize the static member variables outside the class.
// The default values here match those in `main.cpp` for consistency.
float MovingObject::s_neighborhood_max_dist = 10.0f; //  
float MovingObject::s_separation_weight = 0.02f; //  separation_factor
float MovingObject::s_alignment_weight = 0.005f;
float MovingObject::s_cohesion_weight = 0.07f;
float MovingObject::s_boundary_weight = 5.0f;

// float MovingObject::neighborhood_max_dist_ = 10;

float MovingObject::separation_min_dist_ = 1;

// float MovingObject::separation_factor_ = 0.02;
// float MovingObject::cohesion_factor_ = 0.07;
// float MovingObject::alignment_factor_ = 0.005;

float MovingObject::randomness_ = 0;
float MovingObject::max_speed_ = 10.f;

float MovingObject::min_cos_angle_ = -0.5f;

int MovingObject::next_id_ = 0;

// getter function so that other can access these parameters. 
float MovingObject::getNeighborMaxDist() {
    return s_neighborhood_max_dist;
}
float MovingObject::getSeparationWeight() {
    return s_separation_weight;
}
float MovingObject::getCohesionWeight() {
    return s_cohesion_weight;
}
float MovingObject::getAlignmentWeight() {
    return s_alignment_weight;
}

// Implement the static setter methods.
void MovingObject::setNeighborMaxDist(float weight) {
    s_neighborhood_max_dist = weight;
}
void MovingObject::setSeparationWeight(float weight) {
    s_separation_weight = weight;
}

void MovingObject::setAlignmentWeight(float weight) {
    s_alignment_weight = weight;
}

void MovingObject::setCohesionWeight(float weight) {
    s_cohesion_weight = weight;
}

void MovingObject::setBoundaryWeight(float weight) {
    s_boundary_weight = weight;
}
bool MovingObject::are_neighbors(const MovingObject &left, const MovingObject &right)
{
    float temp_neighbor_max_dist = MovingObject::getNeighborMaxDist();
    return (left.get_position() - right.get_position()).norm() < temp_neighbor_max_dist;
    // return (left.get_position() - right.get_position()).norm() < neighborhood_max_dist_;
}

MovingObject::MovingObject(const Vec3f &position, const Vec3f &speed) : id_(next_id_++),
                                                                        position_(position),
                                                                        speed_(speed),
                                                                        boid_type_(-1)
{
}
