#include "queue.hpp"
#include "pointcloud.hpp"

Queue::Queue() {
  this->head = 0;
  this->tail = 0;
}

void Queue::push(const points &frame) {
  while((head + 1)%BUFFER_SIZE == tail);
  buffer[tail++] = frame;
}

points Queue::pop(void) {
  while(head == tail);
  return buffer[head--];
}
