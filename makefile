EXEC=demo
COMP=g++
OBJ= \
	Box.o \
	CustomRigidBody.o \
	Demo.o \
	Display.o \
	Engine.o \
	Force.o \
	Geometry.o \
	Matrix3.o \
	RigidBody.o \
	Simplex.o \
	Sphere.o \
	Vector3.o

$(EXEC): $(OBJ)
	$(COMP) -g -Wall $^ -lglut -lGLU -ggdb -o $(EXEC)

%.o: %.cc
	$(COMP) -o $@ -c $^

clean:
	rm *.o
