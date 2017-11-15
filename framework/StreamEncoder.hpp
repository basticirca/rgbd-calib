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
   * Reconstructs a point cloud 
   * of specified type into pc_.
   * Returns pc_.
  */
  PointCloud* reconstructPointCloud(POINT_CLOUD_TYPE type);

  /* Creates a zmq message from pc8 */
  zmq::message_t createMessage();

  unsigned char* frame;
  PointCloud* pc;
  BoundingBox default_bounding_box;

private:
  /* 
   * Updates frame size according to set frame_size_bytes_.
   * All previously set data in frame will be lost.
  */
  void allocateFrame();

  /* Helper function to ensures pc_ is of given type */
  void ensurePointCloudType(POINT_CLOUD_TYPE type);

  /* creates a PointCloud of given type to pc_ */
  void createPC(POINT_CLOUD_TYPE type);

  size_t frame_size_bytes_;
  size_t clr_size_bytes_;
  size_t depth_size_bytes_;
  size_t header_size_bytes_;
  RGBDSensor* sensor_;
  std::vector<CalibVolume*> cvs_;
  float* header_;
};


#endif // #ifndef  STREAM_ENCODER_HPP

