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
   	Matrix3& operator+=(const Matrix3& m);
    Matrix3 operator-(const Matrix3& m) const;
    Matrix3& operator-=(const Matrix3& m);
 
    Matrix3 operator*(double k) const;
    friend Matrix3 operator*(double k, const Matrix3& m);
    Matrix3 operator*(const Matrix3& m) const;
    Vector3 operator*(const Vector3& v) const;

    friend std::ostream& operator<<(std::ostream& os, const Matrix3& m);

    Matrix3 transpose() const;
    Matrix3 inverse() const;
    Matrix3 normalize() const;
    Matrix3 orthogonalize() const;
    
    Vector3 getRow(int i) const;
    void setRow(int i, Vector3 v);
    Vector3 getColumn(int i) const;
    void setColumn(int i, Vector3 v);

    void reset();
    void identity();
};

#endif

