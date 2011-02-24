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

Vector3 Vector3::operator+(const Vector3& v) const
{
  Vector3 result;

  result.X(this->x + v.X());
  result.Y(this->y + v.Y());
  result.Z(this->z + v.Z());

  return result;
}

Vector3 Vector3::operator-(const Vector3& v) const
{
  Vector3 result;

  result.X(this->x - v.X());
  result.Y(this->y - v.Y());
  result.Z(this->z - v.Z());

  return result;
}

Vector3& Vector3::operator+=(const Vector3& v)
{
  this->X(this->x + v.X());
  this->Y(this->y + v.Y());
  this->Z(this->z + v.Z());

  return *this;
}

Vector3& Vector3::operator-=(const Vector3& v)
{
  this->X(this->x - v.X());
  this->Y(this->y - v.Y());
  this->Z(this->z - v.Z());

  return *this;
}

Vector3 Vector3::operator*(double k) const
{
  Vector3 result;

  result.X(this->x * k);
  result.Y(this->y * k);
  result.Z(this->z * k);

  return result;
}

Vector3 operator*(double k, const Vector3& v)
{
  return v * k;
}

/**
 * Dot product
 */
Vector3 Vector3::operator*(const Vector3& v) const
{
  Vector3 result;

  result.X(this->x * v.X());
  result.Y(this->y * v.Y());
  result.Z(this->z * v.Z());

  return result;
}

/**
 * Cross product
 */
Vector3 Vector3::operator^(const Vector3& v) const
{
  Vector3 result;

  result.X(this->y * v.Z() - this->z * v.Y());
  result.Y(this->x * v.Z() - this->z * v.X());
  result.Z(this->x * v.Y() - this->y * v.X());
  
  return result;
}

/**
 * Cross product with a matrix
 */
Matrix3 Vector3::operator^(const Matrix3& m) const
{
  Matrix3 result;

  result(0, *this ^ m(0));
  result(1, *this ^ m(1));
  result(2, *this ^ m(2));

  return result;
}

std::ostream& operator<<(std::ostream& os, const Vector3& v)
{
  os << "(" << v.x << "," << v.y << "," << v.z << ")";

  return os;
}

double Vector3::length() const
{
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

Vector3 Vector3::normalize() const
{
  Vector3 result = *this;
  
  result = result * (1 / result.length());

  return result;
}

/**
 * Transforms a Vector3 into a special kind of Matrix3 used in the simulation
 */
Matrix3 Vector3::toStarMatrix()
{
  Matrix3 result;

  result(0, 1, this->Z());
  result(0, 2, -this->Y());
  result(1, 0, -this->Z());
  result(1, 2, this->X());
  result(2, 0, this->Y());
  result(2, 1, -this->X());

  return result;
}

void Vector3::reset()
{
	this->x = this->y = this->z = 0.0;
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

int Vector3::size() const
{
  return 3;
}

