#include "StreamEncoder.hpp"

namespace {

  template <class T>
  inline std::string
  toString(T value)
  {
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

}

StreamEncoder::StreamEncoder(RGBDConfig const& config, std::vector<std::string> const& cv_names)
  : frame()
  , pc(nullptr)
  , frame_size_bytes_()
  , clr_size_bytes_()
  , depth_size_bytes_()
  , header_size_bytes_()
  , sensor_(nullptr)
  , cvs_()
  , header_()
{
  sensor_ = new RGBDSensor(config, cv_names.size()-1);
  
  glm::uvec2 cfg_rgb = config.size_rgb;
  glm::uvec2 cfg_d = config.size_d;
  clr_size_bytes_ = cfg_rgb.x * cfg_rgb.y * 3;
  depth_size_bytes_ = cfg_d.x * cfg_d.y * sizeof(float);
  
  frame_size_bytes_ = (clr_size_bytes_ + depth_size_bytes_) * cv_names.size();

  for(unsigned i = 0; i < cv_names.size(); ++i){
    std::string filename_xyz(cv_names[i] + "_xyz");
    std::string filename_uv(cv_names[i] + "_uv");
    cvs_.push_back(new CalibVolume(filename_xyz.c_str(), filename_uv.c_str()));
  }
  
  allocateFrame();

  pc = new PointCloud;

  header_ = new int[1];
  header_size_bytes_ = 1 * sizeof(int);
}


StreamEncoder::~StreamEncoder()
{
  delete [] frame;
  delete sensor_;
  for(unsigned i = 0; i < cvs_.size(); ++i)
    delete cvs_[i];
  delete pc;
  delete [] header_; 
}

PointCloud* StreamEncoder::reconstructPointCloud()
{
  unsigned num_kinect_cameras = sensor_->slave_frames_rgb.size() + 1;
  
  // read color and depth from frame to sensor
  unsigned offset = 0;
  for(unsigned int kinect_idx = 0; kinect_idx < num_kinect_cameras; ++kinect_idx) {
    offset = kinect_idx * (clr_size_bytes_+depth_size_bytes_);
    if(kinect_idx == 0) {
      memcpy((unsigned char*) sensor_->frame_rgb, (unsigned char*) frame + offset, clr_size_bytes_);
      offset += clr_size_bytes_;
      memcpy((unsigned char*) sensor_->frame_d, (unsigned char*) frame + offset, depth_size_bytes_);
    }
    else {
      memcpy((unsigned char*) sensor_->slave_frames_rgb[kinect_idx-1], (unsigned char*) frame + offset, clr_size_bytes_);
      offset += clr_size_bytes_;
      memcpy((unsigned char*) sensor_->slave_frames_d[kinect_idx-1], (unsigned char*) frame + offset, depth_size_bytes_);
    }
  }

  pc->clear();
  
  // reconstruction
  for(unsigned k_num = 0; k_num < num_kinect_cameras; ++k_num){
    // do 3D recosntruction for each depth pixel
    for(unsigned y = 0; y < sensor_->config.size_d.y; ++y){
      for(unsigned x = 0; x < (sensor_->config.size_d.x - 3); ++x){
        const unsigned d_idx = y* sensor_->config.size_d.x + x;
        float d = k_num == 0 ? sensor_->frame_d[d_idx] : sensor_->slave_frames_d[k_num - 1][d_idx];
        if(d < cvs_[k_num]->min_d || d > cvs_[k_num]->max_d)
          continue;
        
        glm::vec3 pos3D;
        glm::vec2 pos2D_rgb;
        
        pos3D = cvs_[k_num]->lookupPos3D( x * 1.0/sensor_->config.size_d.x,
                 y * 1.0/sensor_->config.size_d.y, d);
        glm::vec2 pos2D_rgb_norm = cvs_[k_num]->lookupPos2D_normalized( x * 1.0/sensor_->config.size_d.x, 
                       y * 1.0/sensor_->config.size_d.y, d);
        pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor_->config.size_rgb.x,
            pos2D_rgb_norm.y * sensor_->config.size_rgb.y);
        
        glm::vec3 rgb = sensor_->get_rgb_bilinear_normalized(pos2D_rgb, k_num);
        
        pc->addVoxel(pos3D, rgb);
      }
    }
  }

  // store num points in header
  header_[0] = pc->size();
  
  return pc;
}

zmq::message_t StreamEncoder::createMessage()
{
  size_t voxel_size_bytes = 2 * header_[0] * sizeof(glm::vec3);
  zmq::message_t zmqm(header_size_bytes_ + voxel_size_bytes);

  unsigned offset = 0;
  memcpy( (unsigned char* ) zmqm.data() + offset, (const unsigned char*) header_, header_size_bytes_);
  offset += header_size_bytes_;
  memcpy( (unsigned char* ) zmqm.data() + offset, (const unsigned char*) pc->points.data(), voxel_size_bytes / 2);
  offset += voxel_size_bytes / 2;
  memcpy( (unsigned char* ) zmqm.data() + offset, (const unsigned char*) pc->colors.data(), voxel_size_bytes / 2);
  
  return zmqm;
}

void StreamEncoder::allocateFrame()
{
  delete [] frame;
  frame = new unsigned char[frame_size_bytes_ / sizeof(unsigned char)];
}