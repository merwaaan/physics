EXEC=demo
OBJ= \
	Cube.o \
	Demo.o \
	Display.o \
	Engine.o \
	Force.o \
	Matrix3.o \
	RigidBody.o \
	Vector3.o

$(EXEC): $(OBJ)
	g++ -g $^ -lglut -lGLU -ggdb -o $(EXEC)

%.o: %.cc
	g++ -o $@ -c $^

clean:
	rm *.o *~ .*.swp
