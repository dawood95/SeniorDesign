/*
 * 
 *
 */

#include <iostream>
#include <stdlib.h>

#include "socket.h"

using namespace std;

int main (int argc, char * argv[]) {
  
  if (argc != 3) {
    cout << "Usage : ./client <address> <port>" << endl;
    exit(EXIT_FAILURE);
  }
  
  int server = connectTo(argv[1], argv[2]);

  char c[10] = "";

  for (int i = 0; i < 100; i++) {
    recvMsg(server, c);
    cout << "GOT " << endl;
  }

  return EXIT_FAILURE;
}
