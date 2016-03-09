
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

using namespace std;

int connectTo(char *, char *);
void sendMsg(int, const char *, int );
void recvMsg(int, char *);

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

int connectTo (char * address, char * port) {

  //Get Address Information
  struct addrinfo hints;
  struct addrinfo *result, *rp;
  memset(&hints, 0, sizeof(struct addrinfo));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  if(getaddrinfo(address, port, &hints, &result) != 0){
    perror("getaddrinfo");
    exit(EXIT_FAILURE);
  }

  int clientSocket;
  for (rp = result; rp != NULL; rp = rp->ai_next) {
    clientSocket = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (clientSocket == -1) continue;
    if(connect(clientSocket, rp->ai_addr, rp->ai_addrlen) != -1) break;
    close(clientSocket);
  }

  if (rp == NULL) {
    perror("Failed to connect");
    exit(EXIT_FAILURE);
  }
  
  freeaddrinfo(result);

  return clientSocket;
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
