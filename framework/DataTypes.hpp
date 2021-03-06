#ifndef RGBD_CALIB_DATATYPES_HPP
#define RGBD_CALIB_DATATYPES_HPP


#define CB_WIDTH 7
#define CB_HEIGHT 5

#define GLM_FORCE_RADIANS
#include <glm/vec3.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <fstream>
#include <vector>


class RGBDConfig{
public:
  glm::uvec2 size_rgb;
  glm::uvec2 size_d;

  glm::vec2 principal_rgb;
  glm::vec2 principal_d;

  glm::vec2 focal_rgb;
  glm::vec2 focal_d;

  glm::mat4 eye_d_to_eye_rgb;
  std::string serverport;

  float intrinsic_rgb[9];
  float distortion_rgb[5];

  float intrinsic_d[9];
  float distortion_d[5];

  bool read(const char* ymlfilename);
  void dump();
};

class shape_desc{
public:
  shape_desc(unsigned a, unsigned b, unsigned c, unsigned d);
  unsigned id[4];
};

extern std::ostream& operator << (std::ostream& o, const shape_desc& sd);


class shape_stats{
public:
  std::vector<float> areas;
  std::vector<float> ratiosH;
  std::vector<float> ratiosV;
  std::vector<shape_desc> corners;
};

class Checkerboard{
public:
  glm::mat4 pose_offset;
  std::vector<glm::vec3> points_local;

  bool save_pose_offset(const char* filename);
  bool load_pose_offset(const char* filename);
 
};


extern std::ostream& operator << (std::ostream& o, const glm::uvec3& v);
extern std::ostream& operator << (std::ostream& o, const glm::vec2& v);
extern std::ostream& operator << (std::ostream& o, const glm::vec3& v);
extern std::ostream& operator << (std::ostream& o, const glm::mat4& v);

  class xyz{
  public:
    float x;
    float y;
    float z;
  };

  extern std::ostream& operator << (std::ostream& o, const xyz& v);
  extern xyz interpolate(const xyz& a, const xyz& b, float t);

  class uv{
  public:
    float u;
    float v;
  };

  extern std::ostream& operator << (std::ostream& o, const uv& v);
  extern uv interpolate(const uv& a, const uv& b, float t);

  class xyz_d{
  public:
    double x;
    double y;
    double z;
  };

  class uv_d{
  public:
    double u;
    double v;
  };


extern xyz operator* (const float, const xyz&);
extern uv operator* (const float, const uv&);


extern xyz operator+ (const xyz&, const xyz&);
extern uv operator+ (const uv&, const uv&);
extern uv operator- (const uv&, const uv&);



extern xyz_d operator* (const float, const xyz_d&);
extern uv_d operator* (const float, const uv_d&);


extern xyz_d operator+ (const xyz_d&, const xyz&);
extern uv_d operator+ (const uv_d&, const uv&);


 class samplePoint{
 public:
   float depth;
   uv tex_color;
   uv tex_depth;
   
   xyz pos_offset;
   uv tex_offset;
   glm::vec3 pos_real;
   float quality;
 };

 /// ostream operator
   extern std::ostream& operator<< (std::ostream&, const samplePoint&);

 class CandidateSample{

 public:
   CandidateSample(const float w, const xyz& p_off, const uv& t_off);
   ~CandidateSample();

   float weight;
   xyz pos_off;
   uv tex_off;

 };

 extern bool operator < (const CandidateSample& a, const CandidateSample& b);


 class evaly{

 public:
   glm::vec3 posV;
   double err_3D;
   double err_2D;
   double distN;
   bool vv;
   unsigned bid;
 };


 class board_desc{
 public:
   float id_pos_x;
   float id_pos_y;
   unsigned bid;
 };



float getBilinear(float* data, unsigned width, unsigned height, float x, float y);

xyz getTrilinear(xyz* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z);
uv  getTrilinear(uv* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z);


glm::vec3 calcMean(const std::vector<glm::vec3>& vecs);

void calcMeanSD(std::vector<float>& values, double& mean, double& stdev);

void calcMeanSDMaxMedian(std::vector<float>& values, double& mean, double& stdev, double& ma, double& median);


void calcMeanSD(std::vector<double>& values, double& mean, double& stdev);

void calcMeanSDMaxMedian(std::vector<double>& values, double& mean, double& stdev, double& ma, double& median);


size_t calcNumFrames(std::ifstream& f, size_t fs);



#endif // #ifndef RGBD_CALIB_DATATYPES_HPP

