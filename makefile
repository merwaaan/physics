EXEC=demo
OBJ= \
	Box.o \
	CustomRigidBody.o \
	Demo.o \
	Display.o \
	Engine.o \
	Force.o \
	Matrix3.o \
	RigidBody.o \
	Sphere.o \
	Vector3.o

$(EXEC): $(OBJ)
	g++ -g -Wall $^ -lglut -lGLU -ggdb -o $(EXEC)

%.o: %.cc
	g++ -o $@ -c $^

clean:
	rm *.o *~ .*.swp
