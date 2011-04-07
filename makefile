EXEC=demo
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
	clang -g -Wall $^ -lglut -lGLU -ggdb -o $(EXEC)

%.o: %.cc
	clang -o $@ -c $^

clean:
	rm *.o
