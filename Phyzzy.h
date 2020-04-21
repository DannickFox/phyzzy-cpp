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
#include "Vect2D.h"

struct Mass
{
    double m;
    double r;
    Vect2D pos;
    Vect2D vel;

    Mass(double, double, Vect2D, Vect2D);
    Vect2D move(Vect2D, double);
    Vect2D weight(Vect2D);
};
Mass::Mass(double mass, double radius, Vect2D position, Vect2D velocity)
    : pos(position.x, position.y), vel(velocity.x, velocity.y)
{
    m = mass;
    r = radius;
}
Vect2D Mass::move(Vect2D force, double delta)
{
    vel += force * delta / m;
    pos += vel * delta;
    return pos;
}
Vect2D Mass::weight(Vect2D gravity)
{
    return gravity * m;
}

struct Spring
{
    double k;
    double d;
    double l;
    double w;

    Spring(double, double, double, double);
    Vect2D forceHooke(Mass, Mass);
    Vect2D forceDamp(Mass, Mass);
};
Spring::Spring(double stiffness = 0, double damping = 0, double restlength = 0, double width = 0)
{
    k = stiffness;
    d = damping;
    l = restlength;
    w = width;
}
Vect2D Spring::forceHooke(Mass A, Mass B)
{
    Vect2D AB = A.pos - B.pos;
    return AB.unit() * k * (l - AB.mag());
}
Vect2D Spring::forceDamp(Mass A, Mass B)
{
    Vect2D AB = A.pos - B.pos;
    Vect2D vAB = A.vel - B.vel;
    return vAB.prj(AB) * -d;
}

struct GraphEdge
{
    Spring s;
    GraphEdge(Spring, Mass*, Mass*);
};
GraphEdge::GraphEdge(Spring S, Mass* mA, Mass* mB) : s(S.k, S.d, S.l, S.w) {}
struct GraphNode
{
    Mass m;
    Vect2D F;
    std::vector<GraphNode*> adjN;
    std::vector<GraphEdge*> adjE;
    
    GraphNode(Mass);
    int addAdjacent(GraphNode*, GraphEdge*);
};
GraphNode::GraphNode(Mass mass) : m(mass.m, mass.r, mass.pos, mass.vel)
{
    F.clr();
}
int GraphNode::addAdjacent(GraphNode* n, GraphEdge* e)
{
    adjN.push_back(n);
    adjE.push_back(e);
    return adjN.size();
}


class PhyzzyModel
{
private:
    std::vector<GraphNode*> nodes;
    std::vector<GraphEdge*> edges;
    friend class PhyzzyEnvironment;
public:
    ~PhyzzyModel(void);
    int addNode(Mass);
    int addEdge(int, int, Spring);
    int remNode(int);
    int remEdge(int);

    size_t totalNodes(void);
    size_t totalEdges(void);
    Vect2D centerCoord(void);

    void applySprings(void);
    void updateFrame(double, size_t);

    const Mass& operator [] (const int&);
};
PhyzzyModel::~PhyzzyModel(void)
{
    for (auto& n : nodes)
        delete n;
    nodes.clear();
    for (auto& e : edges)
        delete e;
    edges.clear();
}
int PhyzzyModel::addNode(Mass m)
{
    nodes.push_back(new GraphNode(m));
    return nodes.size();
}
int PhyzzyModel::addEdge(int A, int B, Spring s)
{
    if (A != B && A < nodes.size() && B < nodes.size())
    {
        GraphEdge* e = new GraphEdge(s, &nodes[A]->m, &nodes[B]->m);
        edges.push_back(e);
        nodes[A]->adjN.push_back(nodes[B]);
        nodes[A]->adjE.push_back(e);
        nodes[B]->adjN.push_back(nodes[A]);
        nodes[B]->adjE.push_back(e);
    }
    return edges.size();
}
int PhyzzyModel::remNode(int x)
{
    if (x < nodes.size())
    {
        int* indices = new int[nodes[x]->adjE.size()];
        int j = 0;
        // Find indices of each edge.
        for (auto& e : nodes[x]->adjE)
        {
            int i;
            for (i = 0; i < edges.size(); i++)
            {
                if (edges[i] = e) break;
            }
            if (i < edges.size()) indices[j++] = i;
        }
        for (int i = 0; i < j; i++) remEdge(indices[i]);
        delete[] indices;
        delete nodes[x];
        nodes.erase(nodes.begin() + x);
    }
    return nodes.size();
}
int PhyzzyModel::remEdge(int x)
{
    if (x >= 0 && x < edges.size() && edges.size() > 0);
    {
        // Disconnect nodes by finding and removing from vectors.
        for (auto &n : nodes)
        {
            int i;
            for (i = 0; i < n->adjE.size(); i++)
            {
                if (n->adjE[i] == edges[x]) break;
            }
            if (i < n->adjE.size())
            {
                n->adjE.erase(n->adjE.begin() + i);
                n->adjN.erase(n->adjN.begin() + i);
            }
        }
        delete edges[x];
        edges.erase(edges.begin() + x);
    }
    return edges.size();
}
size_t PhyzzyModel::totalNodes(void)
{
    return nodes.size();
}
size_t PhyzzyModel::totalEdges(void)
{
    return edges.size();
}
void PhyzzyModel::applySprings(void)
{
    for (auto& n : nodes)
    {
        for (int i = 0; i < n->adjN.size(); i++)
        {
            Spring& couple = n->adjE[i]->s;
            Mass& opposite = n->adjN[i]->m;

            n->F += couple.forceHooke(n->m, opposite);
            n->F += couple.forceDamp(n->m, opposite);
        }
    }
}
void PhyzzyModel::updateFrame(double delta, size_t steps = 1)
{
    double dt = delta / steps;
    for (auto& n : nodes)
    {
        for (int i = 0; i < steps; i++)
        {
            n->m.move(n->F, dt);
        }
        n->F.clr(); // Clear forces for next cycle.
    }
}
// Returns reference to mass. Read only.
const Mass& PhyzzyModel::operator [] (const int& i)
{
    return nodes[i]->m;
}

struct Boundary
{
    Vect2D pos;
    Vect2D dir;
    double ks, kd; // Surface friction coefficientes.
    Boundary(Vect2D, Vect2D, double, double); 
};
Boundary::Boundary(Vect2D position, Vect2D direction, double k_static, double k_dynamic)
    : pos(position.x, position.y), dir(direction.unit().x, direction.unit().y)
{
    ks = k_static;
    kd = k_dynamic;    
};

class PhyzzyEnvironment
{
private:
    Vect2D g; // Gravity.
    double Cd; // Drag coefficient.
    std::vector<Boundary> bounds; // Boundaries
public:
    PhyzzyEnvironment(Vect2D, double);
    ~PhyzzyEnvironment(void);
    int addBoundary(Boundary b);
    void enactForces(PhyzzyModel&);
};
PhyzzyEnvironment::PhyzzyEnvironment(Vect2D gravity, double drag)
    : g(gravity.x, gravity.y)
{
    Cd = drag;
}
PhyzzyEnvironment::~PhyzzyEnvironment(void)
{
    bounds.clear();
}
int PhyzzyEnvironment::addBoundary(Boundary b)
{
    bounds.push_back(b);
    return bounds.size();
}
void PhyzzyEnvironment::enactForces(PhyzzyModel& phz)
{
    for (auto& n : phz.nodes)
    {
        n->F += n->m.weight(g) + n->m.vel * -Cd;
    }
}

#endif