#ifndef VECTOR3_H
#define VECTOR3_H

#include <iostream>

class Matrix3;

class Vector3
{
  private:
    double x;
    double y;
    double z;

  public:
    Vector3();
    Vector3(double x, double y, double z);
    ~Vector3();

    Vector3 operator+(const Vector3& v) const;
    Vector3& operator+=(const Vector3& v);
    Vector3 operator-(const Vector3& v) const;
    Vector3& operator-=(const Vector3& v);

    Vector3 operator*(double k) const;
    friend Vector3 operator*(double k, const Vector3& v);
    Vector3& operator*=(double k);

    Vector3 operator/(double k) const;
    double operator*(const Vector3& v) const;
    Vector3 operator^(const Vector3& v) const;

    friend std::ostream& operator<<(std::ostream& os, const Vector3& v);

    double length() const;
    Vector3 normalize() const;
    Matrix3 toStarMatrix() const;

    double X() const;
    double Y() const;
    double Z() const;
    double get(int i) const;

    void X(double x);
    void Y(double y);
    void Z(double z);
    void set(int i, double v);
		void reset();
};

#endif
