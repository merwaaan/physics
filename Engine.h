#ifndef ENGINE_H
#define ENGINE_H

#include <vector>

#include "Constraint.h"
#include "Display.h"
#include "Force.h"
#include "RigidBody.h"

class Engine
{
  private:
    std::vector<RigidBody*> bodies_p;
    std::vector<Force*> environmentalForces_p;
		std::vector<Constraint*> constraints_p;

		double tolerance;

    double timeStep;
		double startingTime;
		double lastUpdateTime;

    Display display;

  public:
    Engine(int* argc, char** argv, double timestep);
    ~Engine();

    void run();
    void update();
    void applyEnvironmentalForces(RigidBody* rb_p, double dt);
		void applyConstraints(double dt);
		void cleanUp();

		Vector3* computeImpulse(Contact contact);
		void emergencyPush(CustomRigidBody* rb1_p, CustomRigidBody* rb2_p);

    double getAbsoluteTime();
    double getLocalTime();
    double getTimeStep();
    bool needUpdate() { return this->getLocalTime() > this->lastUpdateTime + this->timeStep; }
    void reverseTime();

    void addRigidBody_p(RigidBody* rb_p);
    RigidBody* getBody_p(int i);
    int getBodyCount();

    void addEnvironmentalForce_p(Force* force_p);

		void addConstraint_p(Constraint* constraint_p);
		Constraint* getConstraint_p(int i) { return this->constraints_p[i]; }
		int getConstraintCount() { return this->constraints_p.size(); }
		
		Display* getDisplay_p() { return &this->display; }
		bool areBoundingBoxesDrawn() { return this->display.areBoundingBoxesDrawn(); }

		double getTolerance() { return this->tolerance; }
};

#endif

