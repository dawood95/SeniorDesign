#ifndef D3MQ_POINTCLOUD_H
#define D3MQ_POINTCLOUD_H

typedef struct {
  float x;
  float y;
  float z;
  uint8_t color;
} point;

typedef std::vector<point> points;

#endif
