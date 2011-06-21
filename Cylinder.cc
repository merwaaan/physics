#include "Cylinder.h"

Cylinder::Cylinder(double radius, int sides, double height, double mass) :
	radius(radius),
	sides(sides),
	height(height)
{
	double hh = height / 2;
	double m = mass / (2 * sides);	
	double a = (2 * M_PI) / sides;
	
	// Compute points of plane.
	double* x = new double[sides];
	double* z = new double[sides];
	for(int i = 0; i < sides; ++i)
	{
		x[sides-i-1] = radius * cos(i * a); // Reverse the order for CCW drawing.
		z[sides-i-1] = radius * sin(i * a);
	}

	// Add vertices.
	for(int i = 0; i < sides; ++i)
	{
		this->addVertex(i, x[i], -hh, z[i], m);
		this->addVertex(i + sides, x[i], +hh, z[i], m);
	}

	// Link vertices of sides.
	for(int i = 0; i < sides; ++i)
		this->addPolygon(4, (int[]){i, i + sides, (i + 1) % sides + sides, (i + 1) % sides });

	// Link vertices of bottom and top.
	int* bottom = new int[sides];
	int* top = new int[sides];
	for(int i = 0; i < sides; ++i)
	{
		bottom[i] = i;
		top[i] = i + sides;
	}
	this->addPolygon(sides, bottom);
	this->addPolygon(sides, top);

	delete[] x, z, bottom, top;
}

Cylinder::~Cylinder()
{
}

void Cylinder::computeInverseInertiaTensor()
{
  Matrix3 inertiaTensor;

	double m = (1/this->inverseMass) / 12;
	double sr = this->radius * this->radius;
	double sh = this->height * this->height;

  inertiaTensor.set(0, 0, m * (3 * sr + sh));
  inertiaTensor.set(1, 1, (1/this->inverseMass) * sr / 2);
  inertiaTensor.set(2, 2, m * (3 * sr + sh));


  this->inverseInertiaTensor = inertiaTensor.inverse();
}
