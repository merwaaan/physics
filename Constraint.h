#ifndef CONSTRAINT_H
#define CONSTRAINT_H

class RigidBody;

class Constraint
{
  protected:
	  RigidBody* a_p;
	  RigidBody* b_p;

  public:
	  Constraint(RigidBody* a_p, RigidBody* b_p);
	  ~Constraint();

	  virtual void apply(double dt) = 0;
		virtual void draw() = 0;
};

class DistanceConstraint : public Constraint
{
protected:
	double distance;

public:
	DistanceConstraint(RigidBody* a_p, RigidBody* b_p, double distance);
	~DistanceConstraint();

	virtual void apply(double dt);
	virtual void draw();
};

#endif
