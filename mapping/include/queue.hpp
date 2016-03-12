#ifndef D3MQ_QUEUE_H
#define D3MQ_QUEUE_H

#include <atomic>
#include <vector>
#include <array>
#include "pointcloud.hpp"

#define BUFFER_SIZE 256

class Queue {
  std::atomic<int> head, tail;
public:
  std::array<points, BUFFER_SIZE> buffer;
  void push(const points &);
  points pop();  
  Queue();
};
  

#endif
