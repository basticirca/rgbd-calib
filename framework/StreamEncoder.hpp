#ifndef STREAM_ENCODER_HPP
#define STREAM_ENCODER_HPP

#include <DataTypes.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <PointCloud.hpp>
#include <zmq.hpp>

#include <vector>
#include <string>

class StreamEncoder {

public:
  StreamEncoder(RGBDConfig const& config, std::vector<std::string> const& cv_names);
  ~StreamEncoder();

  /* 
   * Reconstructs point cloud 
   * from frame into pc.
   * Returns pc.
  */
  PointCloud* reconstructPointCloud();

  zmq::message_t createMessage();

  unsigned char* frame;
  PointCloud* pc;

private:
  /* 
   * Updates frame size according to set frame_size_bytes_.
   * All previously set data in frame will be lost.
  */
  void allocateFrame();

  size_t frame_size_bytes_;
  size_t clr_size_bytes_;
  size_t depth_size_bytes_;
  size_t header_size_bytes_;
  RGBDSensor* sensor_;
  std::vector<CalibVolume*> cvs_;
  int* header_;
};


#endif // #ifndef  STREAM_ENCODER_HPP

