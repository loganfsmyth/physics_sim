
# Physics Simulator

This is a basic physics simulator written in C++ with SDL and OpenGL.
The physics part itself is reasonably straightforward and uses an RK4 integrator
and follows most of the architecture discussed on [Gaffer On bGames](http://gafferongames.com/game-physics/integration-basics/).

The more complicated part of this simulation is the collision detection algorithms.
I have used the GJK (Gilbert-Johnson-Keerthi) algorithm to detect collisions coupled with the
EPA (Expanding Polytope) algorithm to find the collision surface normals.

Based on those values, I used normal perturbation to find the collision surface points,
and then found the shared regions on the surfaces of each of the two objects colliding.

There is still work to be done on the physics side of things, but the collision detection
has been working flawlessly in my testing.

# TODO

* Get physics working properly for non-rotational cases
* Implement proper rotation collisions with inertia
* Allow for picking up objects and moving them around
* Make adding new items easy.
* The code works fine for a limited number of items, but there are piles
  and piles of optimizations that can be done to all parts of the algorithms
* Clean up the C++ a bit. My C++ is a bit rusty, so I have been passing lots of
  things around by value instead of reference, so that will also slow things down.
