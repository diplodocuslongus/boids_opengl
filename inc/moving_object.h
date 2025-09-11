/*********************************************************************************************************************
 * File : moving_object.h                                                                                            *
 *                                                                                                                   *
 * 2020 Thomas Rouch                                                                                                 *
 *********************************************************************************************************************/

#ifndef MOVING_OBJECT_H
#define MOVING_OBJECT_H

#include <Eigen/Dense>

using Vec3f = Eigen::Vector3f;

class MovingObject
{
private:
    // glm::vec3 position;
    // glm::vec3 velocity;
    // glm::vec3 acceleration;

    // Static member variables to hold the weights for boid behavior.
    // These will be shared by all instances of the MovingObject class.
    static float s_neighborhood_max_dist;
    static float s_separation_weight;
    // static float separation_min_dist_;
    static float s_alignment_weight;
    static float s_cohesion_weight;
    static float s_boundary_weight;
public:
    // static getter
    static float getSeparationWeight();
    static float getNeighborMaxDist();
    static float getCohesionWeight();
    static float getAlignmentWeight();
    // Static setter methods to modify the weights.
    // These are what `main.cpp` will call to configure the simulation.
    static void setNeighborMaxDist(float weight);
    static void setSeparationWeight(float weight);
    static void setAlignmentWeight(float weight);
    static void setCohesionWeight(float weight);
    static void setBoundaryWeight(float weight);

    MovingObject(const Vec3f &position, const Vec3f &speed = Vec3f(0, 0, 0));

    virtual ~MovingObject() = default;

    inline const Vec3f &get_position() const { return position_; }
    inline const Vec3f &get_speed() const { return speed_; }
    inline int get_id() const { return id_; }
    inline int get_type() const { return boid_type_; }

    virtual Vec3f get_exerced_proximity_force(const MovingObject &boid) const = 0;
    virtual void update(float t) = 0;
    virtual void draw() const = 0;

    // static float neighborhood_max_dist_;
    static float separation_min_dist_;
    // static float separation_factor_;
    // static float cohesion_factor_;
    // static float alignment_factor_;
    static float randomness_;
    static float max_speed_;
    static float min_cos_angle_;

    static bool are_neighbors(const MovingObject &left, const MovingObject &right);

protected:
    static int next_id_;

    int id_;
    int boid_type_;

    Vec3f position_;
    Vec3f speed_;
};

#endif // MOVING_OBJECT_H
