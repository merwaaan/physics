#ifndef DISPLAY_H
#define DISPLAY_H

class Engine;

class Display
{
  private:
    Engine* engine_p;

  public:
    Display(int* argc, char** argv, int w, int h, Engine* engine_p);
    ~Display();

    void run();

    Engine* getEngine_p();
};

void update();

#endif
