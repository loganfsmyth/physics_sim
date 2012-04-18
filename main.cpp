
#include "game.h"
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {
  Game g;
  try {
    g.run();
  }
  catch (string &s) {
    cerr << "Err: " << s << endl;
    return 1;
  }
  return 0;
}


