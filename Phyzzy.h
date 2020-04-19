/**
 * PhyzzyModel.h
 * PhyzzyModel 2D spring-mass system library.
 * Inspired by Sodaplay's Sodaconstructor.
 * It's like Soda but it isn't. It's Phyzzy, like Soda,
 * but that's about it.
*/

#ifndef PHYZZY_H
#define PHYZZY_H

#include <vector>
#include <string>
#include "Vect2D.h"

class Mass
{
private:
    double m;   // Mass
    double rad; // Radius.
    Vect2D pos; // Position vector.
    Vect2D vel; // Velocity vector.

    friend class Spring;
    friend class PhyzzyModel;
    friend class PhyzzyEnv;

public:
    Mass(double, double, Vect2D, Vect2D);
    void applyForce(Vect2D, double);
};
Mass::Mass(double mass, double radius, Vect2D position, Vect2D velocity)
    : pos(position.x, position.y), vel(velocity.x, velocity.y)
{
    if (mass < 0) mass = -mass; // Prevents negative mass.
    m = mass;
    rad = radius;
}
void Mass::applyForce(Vect2D f, double delta)
{
    vel += (f * delta) / m;
    pos += vel * delta;
}

class Spring
{
private:
    double stiff;
    double damp;
    double rest;
    double width = 0.1;

public:
    Spring(double, double, double);
    Vect2D forceHooke(Mass, Mass);
    Vect2D forceDampen(Mass, Mass);
};
Spring::Spring(double stiffness, double dampening, double restlength)
{
    stiff = stiffness;
    damp = dampening;
    rest = restlength;
}
// Hooke's law for ideal springs. Output force is applied to A.
Vect2D Spring::forceHooke(Mass A, Mass B)
{
    Vect2D segment = A.pos - B.pos;
    return segment.unit() * stiff * (rest - segment.mag());
}
// Fluid resistance for spring dampening. Output force is applied to A.
Vect2D Spring::forceDampen(Mass A, Mass B)
{
    Vect2D segment = A.pos - B.pos;
    Vect2D relVel = A.vel - B.vel;
    return relVel.prj(segment) * (-damp);
}

// The graph wraps around Mass objects to manage graph such as connections and forces.
struct GraphNode
{
    std::string id; // Node identifier
    
    Mass m; // Contains the mass to be used.
    Vect2D force = Vect2D(); // Force applied to current node.
    std::vector<int> adjNode; // Index of adjacent node.
    std::vector<int> edgSprg; // Index of spring that connects nodes.
    GraphNode(std::string, double, double, Vect2D, Vect2D);
    ~GraphNode(void);
};
GraphNode::GraphNode(std::string nodeID, double mass, double rad, Vect2D pos, Vect2D vel)
    : m(mass, rad, pos, vel)
{
    id = nodeID;
};
GraphNode::~GraphNode(void)
{
    adjNode.clear();
    edgSprg.clear();
}

class PhyzzyModel
{
private:
    std::vector<GraphNode> graph; // Tracks masses and manages the graph.
    std::vector<Spring> springs; // Tracks the springs being used in the graph.
    friend class PhyzzyEnv;
public:
    ~PhyzzyModel(void);
    int addMass(double, double, Vect2D, Vect2D);
    void addSpring(int, int, double, double, double);
    int locateMass(Vect2D, double);
    Vect2D getMassPos(int);
    Vect2D getMassPos(std::string);
    Vect2D getMassVel(int);
    void resetForces(void);
    void applySprings(void);
    void updateFrame(double, int);

    Vect2D& operator [] (const int&);
};
PhyzzyModel::~PhyzzyModel(void)
{
    graph.clear();
    springs.clear();
}
// Add a new mass to the graph. Returns the number of masses in mesh.
int PhyzzyModel::addMass(double mass, double rad, Vect2D pos, Vect2D vel)
{
    std::string nodeID = "m" + std::to_string(graph.size());
    graph.push_back(GraphNode(nodeID, mass, rad, pos, vel));
    return graph.size();
}
// Connect 2 masses with a new spring.
void PhyzzyModel::addSpring(int A, int B, double spr, double dmp, double rest)
{
    // Connect different masses within the graph only.
    if (A != B && A >= 0 && B >= 0 && A < graph.size() && B < graph.size())
    {
        springs.push_back(Spring(spr, dmp, rest));
        // Connect A and B
        graph[A].adjNode.push_back(B);
        graph[B].adjNode.push_back(A);
        // Define spring properties for connection.
        graph[A].edgSprg.push_back(springs.size() - 1);
        graph[B].edgSprg.push_back(springs.size() - 1);
    }
}
// Returns mass nearest to given vector within a radius.
int PhyzzyModel::locateMass(Vect2D point, double radius)
{
    for (int i = 0; i < graph.size(); i++)
    {
        if ((point - graph[i].m.pos).mag() < radius)
        {
            return i;
        }
    }
    return -1;
}
// Returns the position of the current mass.
Vect2D PhyzzyModel::getMassPos(int x)
{
    return graph[x].m.pos;
}
Vect2D PhyzzyModel::getMassPos(std::string x)
{
    Vect2D p;
    
    for (auto &gn : graph)
    {
        if (x.compare(gn.id) == 0)
        {
            p = gn.m.pos;
            break;
        }
    }
    return p;
}
// Returns the velocity of the given mass.
Vect2D PhyzzyModel::getMassVel(int x)
{
    return graph[x].m.vel;
}
// Reset forces.
void PhyzzyModel::resetForces()
{
    for (auto& gn : graph)
    {
        gn.force.clr();
    }
}
// Applies spring forces. Use before any other force application.
void PhyzzyModel::applySprings(void)
{
    // Cycle through graph.
    for (auto& gn : graph)
    {
        gn.force.clr(); // Resets all previous forces.

        // Cycle through adjacent nodes of current node.
        for (int i = 0; i < gn.adjNode.size(); i++)
        {
            // Reference to mass in opposite node.
            Mass& opposite = graph[gn.adjNode[i]].m;
            // Reference to spring that connects mass.
            Spring& couple = springs[gn.edgSprg[i]];

            // Apply springing hooke forces.
            gn.force += couple.forceHooke(gn.m, opposite);
            // Apply springing dampening forces.
            gn.force += couple.forceDampen(gn.m, opposite);
        }
    }
}
// Integrates and updates masses.
void PhyzzyModel::updateFrame(double delta, int steps)
{
    double dt = delta / steps;
    for (auto& gn : graph)
    {
        for (int i = 0; i < steps; i++)
        {
            gn.m.applyForce(gn.force, dt);
        }
    }
}

// Returns position of given mass according to operator. For faster display.
Vect2D& PhyzzyModel::operator [] (const int& i)
{
    return graph[i].m.pos;
} 

struct EnvBound
{
    Vect2D pos; // Boundary center position.
    Vect2D dir; // Boundary direction
    double ks, kd; // Static and dynamic friction coefficients.
    double refl; // Boundary reflection.
    EnvBound(Vect2D, Vect2D, double, double, double);
};
EnvBound::EnvBound(Vect2D position, Vect2D direction,
    double k_static, double k_dyn, double reflect)
{
    pos = position;
    dir = direction.unit();
    ks = k_static;
    kd = k_dyn;
    refl = reflect;
}

// Environment for the model.
class PhyzzyEnv
{
private:
    Vect2D g; // Environment gravity.
    double Cd;  // Environment drag coefficient.
    std::vector<EnvBound> bounds;
    
public:
    PhyzzyEnv(Vect2D, double);
    ~PhyzzyEnv(void);
    void addBound(EnvBound);
    void enactForces(PhyzzyModel&);
};
PhyzzyEnv::PhyzzyEnv(Vect2D gravity, double drag = 0)
{
    g = gravity;
    Cd = drag;
}
PhyzzyEnv::~PhyzzyEnv(void)
{
    bounds.clear();
}
void PhyzzyEnv::enactForces(PhyzzyModel& phz)
{
    for(auto& gn : phz.graph)
    {
        gn.force += g + gn.m.vel * (-Cd);
    }
}
void PhyzzyEnv::addBound(EnvBound b)
{
    bounds.push_back(b);
}

#endif