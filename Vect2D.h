// Vect2D.h

#ifndef VECT2D_H
#define VECT2D_H

#include <cmath>

// 2D vector class.
struct Vect2D
{
    double x, y;
    
    Vect2D(double, double);
    void setCoord(double, double);

    double dot(Vect2D);
    double crs(Vect2D);
    double mag(void);
    double mag2(void);
    double ang(void);
    Vect2D unit(void);
    Vect2D prj(Vect2D);
    void   clr(void);
    Vect2D prp(void);

    Vect2D operator + (const Vect2D&);  // Vector addition.
    Vect2D operator - (const Vect2D&);  // Vector subtraction.
    Vect2D operator * (const double&);  // Vector multiplication with scalar value.
    Vect2D operator / (const double&);  // Vector division with scalar value.
    Vect2D operator += (const Vect2D&); // Mutable vector addition.
    Vect2D operator -= (const Vect2D&); // Mutable vector subtraction.
    Vect2D operator *= (const double&); // Mutable vector multiplication with scalar value.
    Vect2D operator /= (const double&); // Mutable vector division with scalar value.
};
// Class constructor.
Vect2D::Vect2D(double coordX = 0, double coordY = 0)
{
    setCoord(coordX, coordY);
}
// Vector coordinate setter.
void Vect2D::setCoord(double coordX, double coordY)
{
    x = coordX;
    y = coordY;
}
// Scalar dot product.
double Vect2D::dot(Vect2D A)
{
    return x * A.x + y * A.y;
}
// Cross product of vector and given vector.
double Vect2D::crs(Vect2D A)
{
    return x * A.y - y * A.x;
}
// Magnitude squared.
double Vect2D::mag2(void)
{
    return x * x + y * y;
}
// Magnitude.
double Vect2D::mag(void)
{
    return sqrt(x * x + y * y);
}
// Angle.
double Vect2D::ang(void)
{
    return atan2(y, x);
}
// Returns the unit vector.
Vect2D Vect2D::unit(void)
{
    return (*this) / mag();
}
// Projects the vector onto the given vector.
Vect2D Vect2D::prj(Vect2D A)
{
    return A * this->dot(A) / A.mag2();
}
// Set vector to zero.
void Vect2D::clr(void)
{
    x = 0;
    y = 0;
}
// Get a perpendicular unit vector of the current vector.
Vect2D Vect2D::prp(void)
{
    Vect2D P(1, -x / y);
    return P.unit();
}
// Operator overloads.
// Vector addition.
Vect2D Vect2D::operator + (const Vect2D& r)
{
    return Vect2D(this->x + r.x, this->y + r.y);
}
// Vector subraction.
Vect2D Vect2D::operator - (const Vect2D& r)
{
    return Vect2D(this->x - r.x, this->y - r.y);
}
// Vector multiplication with scalar value.
Vect2D Vect2D::operator * (const double& r)
{
    return Vect2D(this->x * r, this->y * r);
}
// Vector division with scalar value.
Vect2D Vect2D::operator / (const double& r)
{
    return Vect2D(this->x / r, this->y / r);
}
// Mutable vector addition.
Vect2D Vect2D::operator += (const Vect2D& r)
{
    this->x += r.x;
    this->y += r.y;
    return *this;
}
// Mutable vector subtraction.
Vect2D Vect2D::operator -= (const Vect2D& r)
{
    this->x -= r.x;
    this->y -= r.y;
    return *this;
}
// Mutable vector multiplication with scalar value.
Vect2D Vect2D::operator *= (const double& r)
{
    this->x *= r;
    this->y *= r;
    return *this;
}
// Mutable vector division with scalar value.
Vect2D Vect2D::operator /= (const double& r)
{
    this->x /= r;
    this->y /= r;
    return *this;
}

#endif