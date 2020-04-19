// example.cpp
/**
 * The example program sets a system of 3 masses
 * and then simulates the movement. The magnitude
 * of the distance between the first and second
 * masses are printed into an output file named
 * "magnitudes.txt"
*/
#include <iostream>
#include <fstream>
#include "Vect2D.h"
#include "Phyzzy.h"

int main (void)
{
    double totalT = 10 * 60;
    int dataPoints = 1000;
    int stepsPerPoint = 10;
    double delta = totalT / dataPoints;

    PhyzzyModel phz;
    PhyzzyEnv env(Vect2D(0, 9.81), 0);
    env.addBound(EnvBound(Vect2D(2.5, 5), Vect2D(1, 0), 0.8, 0.6, 0.9));
    phz.addMass(20, 0.5, Vect2D(0, 0), Vect2D(0, 0));
    phz.addMass(10, 0.5, Vect2D(1, 0), Vect2D(0, 0));
    phz.addMass(50, 0.5, Vect2D(2, 0), Vect2D(0, 0));
    phz.addSpring(0, 1, 0.5, 1.5, 1.1);
    phz.addSpring(1, 2, 0.5, 0.1, 5);

    std::ofstream outFile("magnitudes.txt");
    for (int i = 0; i <= dataPoints; i++)
    {
        outFile << i * delta << " " << (phz[0] - phz[1]).mag() << "\n";
        phz.applySprings();
        env.enactForces(phz);
        phz.updateFrame(delta, stepsPerPoint);
    }
    outFile.close();

    return 0;
}