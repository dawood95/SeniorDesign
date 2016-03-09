#include <iostream>
#include <cstdio>

#include "socket.h"

#include <librealsense/rs.hpp>

using namespace std;

int main (int argc, char * argv[]) {
  
  if (argc != 2) {
    cout << "Usage : ./server <port>" << endl;
    exit(EXIT_FAILURE);
  }
  
  int server = initServer(argv[1]);
  int client = acceptConnection(server);

  for (int i = 0; i < 100; i++) {
    sendMsg(client, "A", 1);
  }

  return EXIT_FAILURE;
}
