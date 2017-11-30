#ifndef RGBD_CALIB_RGBDSENSOR_QUANT_HPP
#define RGBD_CALIB_RGBDSENSOR_QUANT_HPP


#include <DataTypes.hpp>
#include <ChessboardSampling.hpp>


#define GLM_FORCE_RADIANS
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <zmq.hpp>



#include <opencv/cv.h>

#include <string>
#include <vector>




class RGBDSensorQuant{

public:
  RGBDSensorQuant(const RGBDConfig& cfg, unsigned num_of_slaves = 0);
  ~RGBDSensorQuant();

  RGBDConfig config;

  unsigned char* frame_rgb;
  float* frame_d;
  uint8_t* frame_d_8bit;
  float* frame_quant_range;
  unsigned num_slaves;
  glm::vec3 calc_pos_d(float x /* in pixels*/, float y /*in pixels*/, float d /* in meters*/);
  // retrieve 2D pixel coordinates for a given 3D position in front of the sensor in pixels
  glm::vec2 calc_pos_rgb(const glm::vec3& pos_d);

  void recv(bool recvir = false);

  glm::vec3 get_rgb_bilinear_normalized(const glm::vec2& pos_rgb /*in pixels*/, unsigned stream_num = 0);

  glm::mat4 guess_eye_d_to_world(const ChessboardSampling& cbs, const Checkerboard& cb);
  glm::mat4 guess_eye_d_to_world_static(const ChessboardSampling& cbs, const Checkerboard& cb);

  void display_rgb_d();

  std::vector<unsigned char*> slave_frames_rgb;
  std::vector<float*>         slave_frames_d;
  std::vector<uint8_t*>       slave_frames_d_8_bit;
  std::vector<float*>         slave_frames_quant_range;

private:

  zmq::context_t m_ctx;
  zmq::socket_t  m_socket;

  IplImage* m_cv_rgb_image;
  IplImage* m_cv_depth_image;


};


#endif // #ifndef RGBD_CALIB_RGBDSENSOR_QUANT_HPP
