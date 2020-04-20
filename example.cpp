// example.cpp
/**
 * The example program sets a system of 4 masses
 * and then simulates the movement. The positions
 * of the masses are printed to file "points.txt"
*/
#include <iostream>
#include <iomanip>
#include <fstream>
#include "Vect2D.h"
#include "Phyzzy.h"

int main(void)
{
    // Define environment.
    Vect2D gravity(0, 9.81);
    Boundary ground(Vect2D(0, 5), Vect2D(1,0), 0.8, 0.6);
    PhyzzyEnvironment env(gravity, 0.01);
    env.addBoundary(ground);

    // Define model.
    PhyzzyModel phz;
    Vect2D P_m0(2, 2);
    Vect2D P_m1(3, 2);
    Vect2D P_m2(3, 3);
    Vect2D P_m3(2, 3);
    Vect2D v0(0, 0);

    // Construct a 1x1 box.
    phz.addNode(Mass(0.2, 0.025, P_m0, v0));
    phz.addNode(Mass(0.2, 0.025, P_m1, v0));
    phz.addNode(Mass(0.2, 0.025, P_m2, v0));
    phz.addNode(Mass(0.2, 0.025, P_m3, v0));
    phz.addEdge(0, 1, Spring(0.5, 0.5, 1, 0.01));
    phz.addEdge(1, 2, Spring(0.5, 0.5, 1, 0.01));
    phz.addEdge(2, 3, Spring(0.5, 0.5, 1, 0.01));
    phz.addEdge(3, 0, Spring(0.5, 0.5, 1, 0.01));
    phz.addEdge(0, 2, Spring(0.5, 0.5, (P_m0 - P_m2).mag(), 0.01));
    phz.addEdge(1, 3, Spring(0.5, 0.5, (P_m1 - P_m3).mag(), 0.01));

    std::cout << "Nodes: " << phz.totalNodes() << "\tEdges: " << phz.totalEdges() << std::endl;

    double simTime = 20; // Simulation duration in seconds.
    size_t dataPoints = 100;
    size_t stepsPerPoint = 10; // Steps per dataPoint.
    double delta = simTime / dataPoints;
    std::ofstream output("points.txt");
    for (int i = 0; i <= dataPoints; i++)
    {
        output << std::fixed << std::setprecision(3) << i * delta << ": ";
        for (int j = 0; j < phz.totalNodes(); j++)
        {
            output << "m" << j << "(" << phz[j].x << ", " << phz[j].y << ") ";
        }
        output << "\n";

        phz.applySprings();
        env.enactForces(phz);
        phz.updateFrame(delta, stepsPerPoint);
    }
    std::cout << "Nodes: " << phz.totalNodes() << "\tEdges: " << phz.totalEdges() << std::endl;

    return 0;
}