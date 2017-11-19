#include "PointCloudEncoder.hpp"

#include <cassert>
#include <iostream>

PointCloudEncoder::PointCloudEncoder()
  : Encoder()
  , header_size_bytes_()
  , header_()
{
  // num points + bounding box
  header_ = new float[8];
  header_size_bytes_ = 8 * sizeof(float);
}


PointCloudEncoder::~PointCloudEncoder()
{
  delete [] header_; 
}

zmq::message_t PointCloudEncoder::encode(PointCloud<Vec32, Vec32>* point_cloud, Codec codec)
{
  header_[1] = point_cloud->bounding_box.x_min;
  header_[2] = point_cloud->bounding_box.x_max;
  header_[3] = point_cloud->bounding_box.y_min;
  header_[4] = point_cloud->bounding_box.y_max;
  header_[5] = point_cloud->bounding_box.z_min;
  header_[6] = point_cloud->bounding_box.z_max;
  header_[7] = codec;

  if(codec == PC_3x32p_3x8c) {
    PointCloud<Vec32, Vec8> comp_pc(point_cloud->bounding_box);
    compress(point_cloud, &comp_pc);
    header_[0] = comp_pc.size();
    return createMessage(&comp_pc);
  } 
  else if(codec == PC_3x8p_3x8c) {
    PointCloud<Vec8, Vec8> comp_pc(point_cloud->bounding_box);
    compress(point_cloud, &comp_pc);
    header_[0] = comp_pc.size();
    return createMessage(&comp_pc);
  } 
  else if(codec == PC_1x32p_1x32c) {
    PointCloud<uint32_t, uint32_t> comp_pc(point_cloud->bounding_box);
    compress(point_cloud, &comp_pc);
    header_[0] = comp_pc.size();
    return createMessage(&comp_pc);
  }
  else {
    std::cerr << "ERROR: Codec not implemented!" << std::endl;
  }

  return zmq::message_t();
}

bool PointCloudEncoder::decode(zmq::message_t& msg, PointCloud<Vec32, Vec32>* point_cloud)
{
  if(msg.size() < header_size_bytes_)
    return false;

  unsigned offset = 0;
  memcpy( (unsigned char*) header_, (const unsigned char* ) msg.data() + offset, header_size_bytes_);
  offset += header_size_bytes_;
  unsigned num_points = (unsigned) header_[0];

  point_cloud->clear();
  point_cloud->resize(num_points);
  point_cloud->bounding_box.x_min = header_[1];
  point_cloud->bounding_box.x_max = header_[2];
  point_cloud->bounding_box.y_min = header_[3];
  point_cloud->bounding_box.y_max = header_[4];
  point_cloud->bounding_box.z_min = header_[5];
  point_cloud->bounding_box.z_max = header_[6];

  // prepare point cloud
  PointCloudEncoder::Codec codec  = (PointCloudEncoder::Codec) header_[7];
  if(codec == PC_3x32p_3x8c)
  { 
    PointCloud<Vec32, Vec8> recv_pc(point_cloud->bounding_box);
    extractPC(&recv_pc, msg, num_points, offset);
    decompress(&recv_pc, point_cloud);
  }
  else if(codec == PC_3x8p_3x8c)
  {
    PointCloud<Vec8, Vec8> recv_pc(point_cloud->bounding_box);
    extractPC(&recv_pc, msg, num_points, offset);
    decompress(&recv_pc, point_cloud);
  } 
  else if(codec == PC_1x32p_1x32c) { 
    PointCloud<uint32_t, uint32_t> recv_pc(point_cloud->bounding_box);
    extractPC(&recv_pc, msg, num_points, offset);
    decompress(&recv_pc, point_cloud);
  }
  else {
    return false;
  }

  return true;
}

void PointCloudEncoder::compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<Vec32, Vec8>* to_pc)
{
  to_pc->clear();
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    if(!to_pc->bounding_box.contains(from_pc->points[i]))
      continue;
    to_pc->points.push_back(from_pc->points[i]);
    to_pc->colors.push_back(Vec32ToVec8(from_pc->colors[i]));
  }
}

void PointCloudEncoder::compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<Vec8, Vec8>* to_pc)
{
  to_pc->clear();
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    if(!to_pc->bounding_box.contains(from_pc->points[i]))
      continue;
    to_pc->points.push_back(Vec32ToVec8(from_pc->points[i], to_pc->bounding_box));
    to_pc->colors.push_back(Vec32ToVec8(from_pc->colors[i]));
  }
}

void PointCloudEncoder::compress(PointCloud<Vec32, Vec32>* from_pc, PointCloud<uint32_t, uint32_t>* to_pc)
{
  to_pc->clear();
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    if(!to_pc->bounding_box.contains(from_pc->points[i]))
      continue;
    uint32_t p = Vec32ToUInt32(from_pc->points[i], to_pc->bounding_box, 11, 10, 11);
    to_pc->points.push_back(p);
    uint32_t c = Vec32ToUInt32(from_pc->colors[i], 10, 12, 10);
    to_pc->colors.push_back(c);
  }
}

void PointCloudEncoder::decompress(PointCloud<Vec32, Vec8>* from_pc, PointCloud<Vec32, Vec32>* to_pc)
{
  to_pc->clear();
  to_pc->resize(from_pc->size());
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    to_pc->points[i] = from_pc->points[i];
    to_pc->colors[i] = Vec8ToVec32(from_pc->colors[i]);
  }
}

void PointCloudEncoder::decompress(PointCloud<Vec8, Vec8>* from_pc, PointCloud<Vec32, Vec32>* to_pc)
{
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    to_pc->points[i] = Vec8ToVec32(from_pc->points[i], from_pc->bounding_box);
    to_pc->colors[i] = Vec8ToVec32(from_pc->colors[i]);
  }
}

void PointCloudEncoder::decompress(PointCloud<uint32_t, uint32_t>* from_pc, PointCloud<Vec32, Vec32>* to_pc)
{
  to_pc->clear();
  to_pc->resize(from_pc->size());
  for(unsigned i = 0; i < from_pc->size(); ++i) {
    to_pc->points[i] = UInt32ToVec32(from_pc->points[i], from_pc->bounding_box, 11, 10, 11);
    to_pc->colors[i] = UInt32ToVec32(from_pc->colors[i], 10, 12, 10);
  }
}

template<typename P, typename C>
zmq::message_t PointCloudEncoder::createMessage(PointCloud<P, C>* point_cloud)
{
  size_t points_size_bytes = point_cloud->size() * sizeof(P);
  size_t colors_size_bytes = point_cloud->size() * sizeof(C);

  zmq::message_t zmqm(header_size_bytes_ + points_size_bytes + colors_size_bytes);

  unsigned offset = 0;
  memcpy( (unsigned char* ) zmqm.data() + offset, (unsigned char*) header_, header_size_bytes_);
  offset += header_size_bytes_;
  memcpy( (unsigned char* ) zmqm.data() + offset, (unsigned char*) point_cloud->pointsData(), points_size_bytes);
  offset += points_size_bytes;
  memcpy( (unsigned char* ) zmqm.data() + offset, (unsigned char*) point_cloud->colorsData(), colors_size_bytes);
  
  return zmqm;
}

template<typename P, typename C>
void PointCloudEncoder::extractPC(PointCloud<P, C>* pc, zmq::message_t& msg, unsigned num_points, unsigned start_offset)
{
  pc->resize(num_points);

  unsigned bytes_points = pc->size() * sizeof(P);
  unsigned bytes_colors = pc->size() * sizeof(C);
  
  unsigned offset = start_offset;
  memcpy( (unsigned char*) pc->pointsData(), (const unsigned char* ) msg.data() + offset, bytes_points);
  offset += bytes_points;
  memcpy( (unsigned char*) pc->colorsData(), (const unsigned char* ) msg.data() + offset, bytes_colors);
}