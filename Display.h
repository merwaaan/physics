#ifndef DISPLAY_H
#define DISPLAY_H

class Engine;

struct Camera
{
  double radius;
  double angle;
	double height;
  int lastX;
};

class Display
{
  private:
    Camera camera;

    double lastDisplayTime;

    bool drawBoundingBoxes;

  public:
    Display(int* argc, char** argv, int w, int h);
    ~Display();

    void run();

    Camera* getCamera_p() { return &this->camera; }
		void setCamera(double radius, double angle, double height);

    void setLastDisplayTime(double t) { this->lastDisplayTime = t; }
    double getLastDisplayTime() { return this->lastDisplayTime; }

    void setBoundingBoxesDrawn(bool draw);
    bool areBoundingBoxesDrawn();
};

void update();
void mouseMoved(int x, int y);

#endif
