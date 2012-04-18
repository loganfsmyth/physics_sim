
OPTIONS=-lGL -lGLU -lSDL -lboost_thread -ggdb

HEADERS= gameobj.h collision.h vec.h game.h quat.h chull.h
CODE=main.cpp gameobj.cpp collision.cpp vec.cpp game.cpp quat.cpp chull.cpp

build: ${CODE} ${HEADERS}
	clang++ ${CODE} -o physics_sim ${OPTIONS}


test: collision.cpp vec.cpp collision_test.cpp collision.h vec.h chull.cpp chull.h
	clang++ collision.cpp vec.cpp collision_test.cpp chull.cpp -o collision_test ${OPTIONS}
