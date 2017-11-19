#ifndef RECONSTRUCTOR_HPP
#define RECONSTRUCTOR_HPP

#include <DataTypes.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <PointCloud.hpp>

#include <vector>
#include <string>

class Reconstructor {

public:
  enum Codec {
    PC_3x32p_3x8c,
    PC_3x8p_3x8c,
    PC_1x32p_1x32c
  };

public:
  Reconstructor(RGBDConfig const& config, std::vector<std::string> const& cv_names);
  ~Reconstructor();

  /*
   * Reconstructs a point cloud 
   * of specified type into pc.
   * Returns pc.
  */
  PointCloud<Vec32, Vec32>* reconstructPointCloud();

  unsigned char* frame;
  PointCloud<Vec32, Vec32>* pc;
  BoundingBox default_bounding_box;

private:
  /* 
   * Updates frame size according to set frame_size_bytes_.
   * All previously set data in frame will be lost.
  */
  void allocateFrame();

  /* creates a PointCloud to pc_ */
  void createPC();

  size_t frame_size_bytes_;
  size_t clr_size_bytes_;
  size_t depth_size_bytes_;
  RGBDSensor* sensor_;
  std::vector<CalibVolume*> cvs_;
};


#endif // #ifndef  RECONSTRUCTOR_HPP

