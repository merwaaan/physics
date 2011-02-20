#ifndef MATRIX3_H
#define MATRIX3_H

#include <iostream>
#include <cmath>

#include "Vector3.h"

class Matrix3
{
  private:
    double* values;

  public:
    Matrix3();
    ~Matrix3();

    Vector3 operator()(int column) const;
    void operator()(int column, const Vector3& v);
    double operator()(int column, int row) const;
    void operator()(int column, int row, double value);

    Matrix3& operator=(const Matrix3& m);

    Matrix3 operator+(const Matrix3& m) const;
    Matrix3 operator-(const Matrix3& m) const;
   	Matrix3& operator+=(const Matrix3& m);
    Matrix3& operator-=(const Matrix3& m);
    
 
    Matrix3 operator*(double k) const;
    Matrix3 operator*(const Matrix3& m) const;
    Vector3 operator*(const Vector3& v) const;

    friend std::ostream& operator<<(std::ostream& os, const Matrix3& m);

    Matrix3 transpose() const;
    void normalize();
};

#endif

