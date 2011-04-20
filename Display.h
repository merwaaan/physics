#ifndef DISPLAY_H
#define DISPLAY_H

class Engine;

struct Camera
{
  double radius;
  double angle;

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

    void setBoundingBoxesDrawn(bool draw);

    bool areBoundingBoxesDrawn();

    friend void update();
    friend void mouse(int x, int y);
};

void update();
void mouse(int x, int y);

#endif
