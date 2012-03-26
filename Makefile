
OPTIONS=-lGL -lGLU -lSDL -lboost_thread

HEADERS= gameobj.h collision.h vec.h game.h quat.h chull.h
CODE=main.cpp gameobj.cpp collision.cpp vec.cpp game.cpp quat.cpp chull.cpp

build: ${CODE} ${HEADERS}
	g++ ${CODE} -o physics_sim ${OPTIONS}

