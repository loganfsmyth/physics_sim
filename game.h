
#include <list>
#include "gameobj.h"

class Game {
  // Bool for signaling gameloop exit
  bool close; 

  // camera view angles;
  double leftright;
  double updown;

  // Camera object
  gameobj* camera;

  gameobj* diff;

  // Objects in scene
  std::list<gameobj*> objs;

  // Flags for arrow keys
  bool movement[6];

  // Size of window.
  int width, height;

  // FPS
  int FPS;

  public:
  Game();
  ~Game();

  // Primary game loop
  void run();

  // Simulate next timestep
  void simulate(unsigned long step);

  // Render all elements
  void render(int interp_percent);

  // Process any events in the queue
  void check_events();

  // Handle window resize events
  void resize(int h, int w);

  // Pick element and given x,y position
  void pick(int x, int y);
};
