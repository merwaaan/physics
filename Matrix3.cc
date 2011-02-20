#include "Matrix3.h"

#include "Vector3.h"

Matrix3::Matrix3()
{
  this->values = new double[9];

  int i;
  for(i = 0; i < 9; ++i)
    this->values[i] = 0;
}

Matrix3::~Matrix3()
{
  delete this->values;
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

Matrix3 Matrix3::operator-(const Matrix3& m) const
{
  Matrix3 result;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      result(j, i, (*this)(j, i) - m(j, i));

  return result;
}

Matrix3& Matrix3::operator+=(const Matrix3& m)
{
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      (*this)(j, i, (*this)(j, i) + m(j, i));

  return *this;
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

void Matrix3::normalize()
{
  for(int i = 0; i < 3; ++i)
    (*this)(i, (*this)(i).normalize());
}

