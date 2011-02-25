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

    Engine* engine_p;

  public:
    Display(int* argc, char** argv, int w, int h, Engine* engine_p);
    ~Display();

    void run();

    friend void update();
    friend void mouse(int x, int y);
};

void update();
void mouse(int x, int y);

#endif
