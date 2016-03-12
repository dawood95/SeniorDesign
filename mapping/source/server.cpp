/* 
 * 0. Listen for a connection on the specified port
 * 1. Read data from camera, create point cloud mapping. 
 * 2. Convert point cloud data to custom data structure
 * 3. Send the custom data through TCP
 * 4. Goto 4
 */

#include <iostream>
#include <cstdio>
#include <thread>

#include "socket.hpp"
#include "queue.hpp"
#include "pointcloud.hpp"

#include <librealsense/rs.hpp>

using namespace std;

void cameraThread(rs::device &);

int main (int argc, char * argv[]) {
  
  if (argc != 2) {
    cout << "Usage : ./server <port>" << endl;
    exit(EXIT_FAILURE);
  }
  
  rs::log_to_console(rs::log_severity::warn);
  rs::log_to_file(rs::log_severity::debug, "d3mqServer.log");

  rs::context ctx;
  if(ctx.get_device_count() == 0)
    throw runtime_error("No device detected. Is it plugged in ?");
  rs::device &dev = *ctx.get_device(0);

  int server = initServer(argv[1]);
  int client = acceptConnection(server);

  Queue messageQueue;

  dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
  dev.enable_stream(rs::stream::color, rs::preset::best_quality);
  dev.enable_stream(rs::stream::infrared, rs::preset::best_quality);
  dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality);
  dev.start();
  
  std::thread camera (cameraThread, dev, messageQueue);
  std::thread socket (socketThread, client, messageQueue);

  return EXIT_FAILURE;
}

void cameraThread(rs::device &dev, Queue &msgQ) {
  while(true) {
    // get frames
    if(dev.is_streaming()) dev.wait_for_frames();
    const float depth_scale = dev.get_depth_scale();
    const rs::extrinsics extrin = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
    const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
    const rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);    
    // transform frames to pointcloud
    for (int y = 0; y < depth_intrin.height; ++y) {
      for (int x = 0; x < depth_intrin.width; ++x) {
	uint16_t depth = depth_image[y * depth_intrin.width + x];
	float depth_meters = depth * depth_scale;

	if(depth_value == 0) continue;


      }
    }

    // convert pointcloud
    // put it on transmit queue
  }
}

void socketThread(int client, Queue &msgQ) {
  while(true) {
    
  }
}
