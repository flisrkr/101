#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL
{

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k,
               vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`,
        // and containing `num_nodes` nodes.
        Vector2D step = (end - start) / static_cast<double>(num_nodes - 1);
        Vector2D pos_init = start;
        for (int i = 0; i < num_nodes; i++)
        {
            masses.push_back(new Mass(pos_init, node_mass, false));
            pos_init += step;
        }
        for (int i = 0; i < num_nodes - 1; i++)
        {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }
        //        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        double k_d = 0.1;

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D diff = s->m2->position - s->m1->position;
            Vector2D force = s->k * diff.unit() * (diff.norm() - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity,
                // then compute the new velocity and position
                // m->forces += m->mass * gravity;
                // // TODO (Part 2): Add global damping
                // m->forces += -k_d * m->velocity;

                // Vector2D accel = m->forces / m->mass;

                // // semi-implicit
                // m->velocity += accel * delta_t;
                // m->position += m->velocity * delta_t;

                // explicit
                //  m->position += m->velocity * delta_t;
                //  m->velocity += accel * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        double damping_factor = 0.00005;
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope
            // using explicit Verlet （solving constraints)
            Vector2D diff = s->m2->position - s->m1->position;
            Vector2D force = s->k * diff.unit() * (diff.norm() - s->rest_length);
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->forces += m->mass * gravity;
                Vector2D accel = m->forces / m->mass;

                m->position = temp_position +
                              (1 - damping_factor) * (temp_position - m->last_position) +
                              accel * delta_t * delta_t;

                m->last_position = temp_position;
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
