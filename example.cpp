// example.cpp
/**
 * The example program sets a system of 2 masses
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
    phz.addMass(20, Vect2D(0, 0), Vect2D(0, 0));
    phz.addMass(10, Vect2D(1, 0), Vect2D(0, 0));
    phz.addMass(50, Vect2D(2, 0), Vect2D(0, 0));
    phz.addSpring(0, 1, 0.5, 1.5, 1.1);
    phz.addSpring(1, 2, 0.5, 0.1, 5);

    std::ofstream outFile("magnitudes.txt");
    for (int i = 0; i <= dataPoints; i++)
    {
        outFile << i * delta << " " << (phz[0] - phz[1]).mag() << "\n";
        phz.applySprings();
        phz.updateFrame(delta, stepsPerPoint);
    }
    outFile.close();

    return 0;
}