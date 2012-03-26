
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
