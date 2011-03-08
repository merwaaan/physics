#include "Matrix3.h"

#include "Vector3.h"

Matrix3::Matrix3()
{
  this->values = new double[9];

  for(int i = 0; i < 9; ++i)
    this->values[i] = 0;
}

Matrix3::~Matrix3()
{
  //delete this->values;
}

Matrix3& Matrix3::operator=(const Matrix3& m)
{
  for(int i = 0; i < 9; ++i)
    this->values[i] = m.values[i];

  return *this;
}

Matrix3 Matrix3::operator+(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 9; ++i)
    result.values[i] = this->values[i] + m.values[i];

  return result;
}

Matrix3& Matrix3::operator+=(const Matrix3& m)
{
  for(int i = 0; i < 9; ++i)
    this->values[i] += m.values[i];

  return *this;
}

Matrix3 Matrix3::operator-(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 9; ++i)
    result.values[i] = this->values[i] - m.values[i];

  return result;
}

Matrix3& Matrix3::operator-=(const Matrix3& m)
{
  for(int i = 0; i < 9; ++i)
    this->values[i] -= m.values[i];

  return *this;
}

Matrix3 Matrix3::operator*(double k) const
{
  Matrix3 result;

  for(int i = 0; i < 9; ++i)
    result.values[i] = this->values[i] * k;

  return result;
}

Matrix3 operator*(double k, const Matrix3& m)
{
  return m * k;
}

Matrix3 Matrix3::operator*(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result.set(j, i, this->get(0, i) * m.get(j, 0) + this->get(1, i) * m.get(j, 1) + this->get(2, i) * m.get(j, 2));

  return result;
}

Vector3 Matrix3::operator*(const Vector3& v) const
{
  Vector3 result;

  result.X(this->get(0, 0) * v.X() + this->get(1, 0) * v.Y() + this->get(2, 0) * v.Z());
  result.Y(this->get(0, 1) * v.X() + this->get(1, 1) * v.Y() + this->get(2, 1) * v.Z());
  result.Z(this->get(0, 2) * v.X() + this->get(1, 2) * v.Y() + this->get(2, 2) * v.Z());

  return result;
}

std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
  for(int i = 0; i < 3; ++i)
  {
    os << "| ";

    for(int j = 0; j < 3; ++j)
      os << m.get(j, i) << " ";

    os << std::endl;
  }

  return os;
}

Matrix3 Matrix3::transpose() const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j <  3; ++j)
      result.set(j, i, this->get(i, j));

  return result;
}

Matrix3 Matrix3::inverse() const
{
  Matrix3 t = *this;

  // find the determinant
  double d =
    t.get(0, 0) * (t.get(1, 1) * t.get(2, 2) - t.get(1, 2) * t.get(2, 1)) -
    t.get(1, 0) * (t.get(0, 1) * t.get(2, 2) - t.get(0, 2) * t.get(2, 1)) +
    t.get(2, 0) * (t.get(0, 1) * t.get(1, 2) - t.get(0, 2) * t.get(1, 1));
  
  // find the matrix of cofactors
  Matrix3 result;
  t = t.transpose();
  result.set(0, 0, t.get(1, 1) * t.get(2, 2) - t.get(1, 2) * t.get(2, 1));
  result.set(0, 1, t.get(1, 0) * t.get(2, 2) - t.get(1, 2) * t.get(2, 0));
  result.set(0, 2, t.get(1, 0) * t.get(2, 1) - t.get(1, 1) * t.get(2, 0));
  result.set(1, 0, t.get(0, 1) * t.get(2, 2) - t.get(0, 2) * t.get(2, 1));
  result.set(1, 1, t.get(0, 0) * t.get(2, 2) - t.get(0, 2) * t.get(2, 0));
  result.set(1, 2, t.get(0, 0) * t.get(2, 1) - t.get(0, 1) * t.get(2, 0));
  result.set(2, 0, t.get(0, 1) * t.get(1, 2) - t.get(0, 2) * t.get(1, 1));
  result.set(2, 1, t.get(0, 0) * t.get(1, 2) - t.get(0, 2) * t.get(1, 0));
  result.set(2, 2, t.get(0, 0) * t.get(1, 1) - t.get(0, 1) * t.get(1, 0));

  // apply sign changes
  result.set(0, 1, -result.get(0, 1));
  result.set(1, 0, -result.get(1, 0));
  result.set(1, 2, -result.get(1, 2));
  result.set(2, 1, -result.get(2, 1));
  
  return result * (1 / d);
}

Matrix3 Matrix3::normalize() const
{
  Matrix3 result = *this;

  for(int c = 0; c < 3; ++c)
    result.setColumn(c, result.getColumn(c).normalize());

  return result;
}

/**
 * Iteratively orthonalize a matrix using the Gram-Schmidt algorithm
 */
Matrix3 Matrix3::orthogonalize() const
{
  int iterations = 10;
  double k = 0.25;

  Vector3 r1 = this->getRow(0);
  Vector3 r2 = this->getRow(1);
  Vector3 r3 = this->getRow(2);

  // iterative orthogonalization
  Vector3 r1b, r2b, r3b;
  for(int i = 0; i < iterations; ++i)
  {
    r1b = r1 - k * ((r1 * r2) / (r2 * r2)) * r2 - k * ((r1 * r3) / (r3 * r3)) * r3;
    r2b = r2 - k * ((r2 * r1) / (r1 * r1)) * r1 - k * ((r2 * r3) / (r3 * r3)) * r3;
    r3b = r3 - k * ((r3 * r2) / (r2 * r2)) * r1 - k * ((r3 * r2) / (r2 * r2)) * r2;

    r1 = r1b;
    r2 = r2b;
    r3 = r3b;
  }

  // final step
  r1b = r1;
  r2b = r2 - ((r2 * r1b) / (r1b * r1b)) * r1b;
  r3b = r3 - ((r3 * r1b) / (r1b * r1b)) * r1b - ((r3 * r2b) / (r2b * r2b)) * r2b;

  Matrix3 result;
  result.setRow(0, r1b);
  result.setRow(1, r2b);
  result.setRow(2, r3b);

  return result;
}

double Matrix3::get(int c, int r) const
{
  return this->values[r * 3 + c];
}

void Matrix3::set(int c, int r, double v)
{
  this->values[r * 3 + c] = v;
}

Vector3 Matrix3::getRow(int r) const
{
  int start = r * 3;

  return Vector3(this->values[start], this->values[start + 1], this->values[start + 2]);
}

void Matrix3::setRow(int r, Vector3 v)
{
  int start = r * 3;

  this->values[start] = v.get(0);
  this->values[start + 1] = v.get(1);
  this->values[start + 2] = v.get(2);
}

Vector3 Matrix3::getColumn(int c) const
{
  return Vector3(this->values[c], this->values[c + 3], this->values[c + 6]);
}

void Matrix3::setColumn(int c, Vector3 v)
{
  this->values[c] = v.get(0);
  this->values[c + 3] = v.get(1);
  this->values[c + 6] = v.get(2);
}

void Matrix3::reset()
{
  for(int i = 0; i < 9; ++i)
    this->values[i] = 0;
}

void Matrix3::identity()
{
  this->reset();

  this->values[0] = 1;
  this->values[4] = 1;
  this->values[7] = 1;
}
