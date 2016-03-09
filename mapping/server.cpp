
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

using namespace std;

int initServer(char *);
int acceptConnection (int );
void sendMsg(int, const char *, int );
void recvMsg(int, char *);

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

int acceptConnection (int masterSocket) {

  struct sockaddr_in clientIPAddress;

  int clientAddrLen = sizeof(clientIPAddress);
  memset(&clientIPAddress, 0, sizeof(clientIPAddress));

  int slaveSocket = accept(masterSocket, (struct sockaddr*) &clientIPAddress, (socklen_t *) &clientAddrLen);

  if (slaveSocket < 0) {
    perror("Bad Socket");
    exit(EXIT_FAILURE);
  }

  return slaveSocket;
}

int initServer (char * port) {

  struct addrinfo hints;
  struct addrinfo *result, *rp;
  memset(&hints, 0, sizeof(struct addrinfo));

  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  if(getaddrinfo(NULL, port, &hints, &result) != 0){
    perror("getaddrinfo");
    exit(EXIT_FAILURE);
  }

  int serverSocket;
  for (rp = result; rp != NULL; rp = rp->ai_next) {
    serverSocket = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (serverSocket == -1) continue;
    int optVal = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &optVal, sizeof(int));
    if (bind(serverSocket, rp->ai_addr, rp->ai_addrlen) == 0) break;
    close(serverSocket);
  }

  if (rp == NULL) {
    perror("Could not bind");
    exit(EXIT_FAILURE);
  }

  freeaddrinfo(result);

  if (listen(serverSocket, 1) == -1) {
    perror("Listen Failed");
    exit(EXIT_FAILURE);
  }

  return serverSocket;
}

void sendMsg (int fd, const char * buffer, int len) {
  int n = 0;
  while( n < sizeof(len))
    n += write(fd, (int *) &len, sizeof(len) - n);
  n = 0;
  while( n < len)
    n += write(fd, buffer, len - n);
  //  std::cout << n << std::endl;
}

void recvMsg (int fd, char * buffer) {
  int len;
  int n = 0;
  while(n < sizeof(len))
    n += read(fd, &len, sizeof(len) - n);
  n = 0;
  while(n < len)
    n += read(fd, buffer, len - n);
}
