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

Vector3 Matrix3::operator()(int column) const
{
  return Vector3((*this)(column, 0), (*this)(column, 1), (*this)(column, 2));
}

void Matrix3::operator()(int column, const Vector3& v)
{
  (*this)(column, 0, v.X());
  (*this)(column, 1, v.Y());
  (*this)(column, 2, v.Z());
}

double Matrix3::operator()(int column, int row) const
{
  return this->values[3 * row + column];
}

void Matrix3::operator()(int column, int row, double value)
{
  this->values[3 * row + column] = value;
}

Matrix3& Matrix3::operator=(const Matrix3& m)
{
  for(int i = 0; i < 9; ++i)
    this->values[i] = m(i % 3, i / 3);

  return *this;
}

Matrix3 Matrix3::operator+(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result(j, i, (*this)(j, i) + m(j, i));

  return result;
}

Matrix3& Matrix3::operator+=(const Matrix3& m)
{
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      (*this)(j, i, (*this)(j, i) + m(j, i));

  return *this;
}

Matrix3 Matrix3::operator-(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result(j, i, (*this)(j, i) - m(j, i));

  return result;
}

Matrix3& Matrix3::operator-=(const Matrix3& m)
{
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      (*this)(j, i, (*this)(j, i) - m(j, i));

  return *this;
}

Matrix3 Matrix3::operator*(double k) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result(j, i, (*this)(j, i) * k);

  return result;
}

Matrix3 operator*(double k, const Matrix3& m)
{
  return m * k;
}

// TODO
Matrix3 Matrix3::operator*(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result(j, i, (*this)(0, i) * m(j, 0) + (*this)(1, i) * m(j, 1) + (*this)(2, i) * m(j, 2));

  return result;
}

Vector3 Matrix3::operator*(const Vector3& v) const
{
  Vector3 result;

  result.X((*this)(0, 0) * v.X() + (*this)(1, 0) * v.Y() + (*this)(2, 0) * v.Z());
  result.Y((*this)(0, 1) * v.X() + (*this)(1, 1) * v.Y() + (*this)(2, 1) * v.Z());
  result.Z((*this)(0, 2) * v.X() + (*this)(1, 2) * v.Y() + (*this)(2, 2) * v.Z());

  return result;
}

std::ostream& operator<<(std::ostream& os, const Matrix3& m)
{
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
      os << m(j, i) << " ";

    os << std::endl;
  }

  return os;
}

Matrix3 Matrix3::transpose() const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j <  3; ++j)
      result(j, i, (*this)(i, j));

  return result;
}

Matrix3 Matrix3::inverse() const
{
  Matrix3 t = *this;

  // find the determinant
  double d =
    t(0, 0) * (t(1, 1) * t(2, 2) - t(1, 2) * t(2, 1)) -
    t(1, 0) * (t(0, 1) * t(2, 2) - t(0, 2) * t(2, 1)) +
    t(2, 0) * (t(0, 1) * t(1, 2) - t(0, 2) * t(1, 1));
  
  // find the matrix of cofactors
  Matrix3 result;
  t = t.transpose();
  result(0, 0, t(1, 1) * t(2, 2) - t(1, 2) * t(2, 1));
  result(0, 1, t(1, 0) * t(2, 2) - t(1, 2) * t(2, 0));
  result(0, 2, t(1, 0) * t(2, 1) - t(1, 1) * t(2, 0));
  result(1, 0, t(0, 1) * t(2, 2) - t(0, 2) * t(2, 1));
  result(1, 1, t(0, 0) * t(2, 2) - t(0, 2) * t(2, 0));
  result(1, 2, t(0, 0) * t(2, 1) - t(0, 1) * t(2, 0));
  result(2, 0, t(0, 1) * t(1, 2) - t(0, 2) * t(1, 1));
  result(2, 1, t(0, 0) * t(1, 2) - t(0, 2) * t(1, 0));
  result(2, 2, t(0, 0) * t(1, 1) - t(0, 1) * t(1, 0));

  // apply sign changes
  result(0, 1, -result(0, 1));
  result(1, 0, -result(1, 0));
  result(1, 2, -result(1, 2));
  result(2, 1, -result(2, 1));
  
  return result * (1 / d);
}

Matrix3 Matrix3::normalize() const
{
  Matrix3 result = *this;

  for(int i = 0; i < 3; ++i)
    result(i, result(i).normalize());

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

Vector3 Matrix3::getRow(int i) const
{
  int start = i * 3;

  return Vector3(this->values[start], this->values[start + 1], this->values[start + 2]);
}

void Matrix3::setRow(int i, Vector3 v)
{
  int start = i * 3;

  this->values[start] = v.get(0);
  this->values[start + 1] = v.get(1);
  this->values[start + 2] = v.get(2);
}

Vector3 Matrix3::getColumn(int i) const
{
  return Vector3(this->values[i], this->values[i + 3], this->values[i + 6]);
}

void Matrix3::setColumn(int i, Vector3 v)
{
  this->values[i] = v.get(0);
  this->values[i + 3] = v.get(1);
  this->values[i + 6] = v.get(2);
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
