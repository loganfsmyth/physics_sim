
OPTIONS=-lGL -lGLU -lSDL

build: main.cpp gameobj.cpp collision.cpp vec.cpp
	g++ main.cpp gameobj.cpp collision.cpp vec.cpp -o physics_sim ${OPTIONS}

