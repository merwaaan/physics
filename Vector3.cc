#include "Vector3.h"

#include <cmath>

#include "Matrix3.h"

Vector3::Vector3() :
  x(0),
  y(0),
  z(0)
{
}

Vector3::Vector3(double x, double y, double z) :
  x(x),
  y(y),
  z(z)
{
}

Vector3::~Vector3()
{
}

bool Vector3::operator==(const Vector3& v) const
{
	double tolerance = 0.001;

	return (*this - v).length() < tolerance;
}

Vector3 Vector3::operator+(const Vector3& v) const
{
  Vector3 result;

  result.x = this->x + v.x;
  result.y = this->y + v.y;
  result.z = this->z + v.z;

  return result;
}

Vector3& Vector3::operator+=(const Vector3& v)
{
  this->x += v.x;
  this->y += v.y;
  this->z += v.z;

  return *this;
}

Vector3 Vector3::operator-(const Vector3& v) const
{
  Vector3 result;

  result.x = this->x - v.x;
  result.y = this->y - v.y;
  result.z = this->z - v.z;

  return result;
}

Vector3& Vector3::operator-=(const Vector3& v)
{
  this->x -= v.x;
  this->y -= v.y;
  this->z -= v.z;

  return *this;
}

Vector3 Vector3::operator*(double k) const
{
  Vector3 result;

  result.x = this->x * k;
  result.y = this->y * k;
  result.z = this->z * k;

  return result;
}

Vector3 operator*(double k, const Vector3& v)
{
  return v * k;
}

Vector3& Vector3::operator*=(double k)
{
  this->x *= k;
  this->y *= k;
  this->z *= k;

  return *this;
}

Vector3 Vector3::operator/(double k) const
{
  Vector3 result;

  result.x = this->x / k;
  result.y = this->y / k;
  result.z = this->z / k;

  return result;
}

/**
 * Dot product
 */
double Vector3::operator*(const Vector3& v) const
{
  return this->x * v.x + this->y * v.y + this->z * v.z;
}

/**
 * Cross product
 */
Vector3 Vector3::operator^(const Vector3& v) const
{
  Vector3 result;

  result.x = this->y * v.z - this->z * v.y;
  result.y = this->x * v.z - this->z * v.x;
  result.z = this->x * v.y - this->y * v.x;
  
  return result;
}

std::ostream& operator<<(std::ostream& os, const Vector3& v)
{
  os << "[" << v.x << "," << v.y << "," << v.z << "]";

  return os;
}

double Vector3::length() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

Vector3 Vector3::normalize() const
{
  Vector3 result = *this;
  
  result *= (1 / result.length());

  return result;
}

/**
 * Transforms a Vector3 into a special kind of Matrix3 used in the simulation
 */
Matrix3 Vector3::toStarMatrix() const
{
  Matrix3 result;

  result.set(0, 1, this->z);
  result.set(0, 2, -this->y);
  result.set(1, 0, -this->z);
  result.set(1, 2, this->x);
  result.set(2, 0, this->y);
  result.set(2, 1, -this->x);

  return result;
}

double Vector3::X() const
{
  return this->x;
}

double Vector3::Y() const
{
  return this->y;
}

double Vector3::Z() const
{
  return this->z;
}

double Vector3::get(int i) const
{
  if(i == 0)
    return this->x;
  
  if(i == 1)
    return this->y;
  
  return this->z;
}
void Vector3::X(double x)
{
  this->x = x;
}

void Vector3::Y(double y)
{
  this->y = y;
}

void Vector3::Z(double z)
{
  this->z = z;
}

void Vector3::set(int i, double v)
{
  if(i == 0)
    this->x = v;
  else if(i == 1)
    this->y = v;
  else
    this->z = v;
}

void Vector3::reset()
{
	this->x = this->y = this->z = 0.0;
}

