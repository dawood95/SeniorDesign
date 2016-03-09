#include "zmq.h"
#include <iostream>

using namespace std;

void free_data (void * data, void * hint) {
  return;
}

int main() {

  zmq_msg_t msg;
  void * context = zmq_ctx_new();
  void * socket = zmq_socket(context, ZMQ_PUSH);

  zmq_bind(socket, "tcp://0.0.0.0:9090");

  for (int i = 0; i < 100; i++) {
    zmq_send(&socket, "A", 1, 0);
  }

  zmq_close(socket);
  zmq_ctx_destroy(context);
  return 0;
}
