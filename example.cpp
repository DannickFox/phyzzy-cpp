// example.cpp
/**
 * The example program sets a system of 4 masses
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

    Vect2D P_m0(2.01, 2);
    Vect2D P_m1(3, 2);
    Vect2D P_m2(3, 3);
    Vect2D P_m3(2, 3);
    Vect2D v0(0, 0);

    phz.addMass(0.1, 0.05, P_m0, v0);
    phz.addMass(0.1, 0.05, P_m1, v0);
    phz.addMass(0.1, 0.05, P_m2, v0);
    phz.addMass(0.1, 0.05, P_m3, v0);
    phz.addSpring(0, 1, 0.8, 0.5, 1);
    phz.addSpring(1, 2, 0.8, 0.5, 1);
    phz.addSpring(2, 3, 0.8, 0.5, 1);
    phz.addSpring(3, 0, 0.8, 0.5, 1);
    phz.addSpring(0, 2, 0.8, 0.5, (P_m0 - P_m2).mag());
    phz.addSpring(1, 3, 0.8, 0.5, (P_m1 - P_m3).mag());

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