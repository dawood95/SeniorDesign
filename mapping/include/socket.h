#ifndef D3MQ_SOCKET_H
#define D3MQ_SOCKET_H

int initServer(char *);

int acceptConnection (int );
int connectTo(char *, char *);

void sendMsg(int, const char *, int );
void recvMsg(int, char *);

#endif
