#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <timevalue.hpp>
#include <clock.hpp>
#include <zmq.hpp>
#include <math.h>
#include <PointCloudEncoder.hpp>
#include <Reconstructor.hpp>
#include <PointCloud.hpp>
#include <PlyParser.hpp>
#include <PlyFrameBuffer.hpp>

#include <iostream>
#include <sstream>

namespace{

  template <class T>
  inline std::string
  toString(T value){
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

}

int main(int argc, char* argv[]){
  std::cout << "Start file\n";

  std::string socket_ip = "127.0.0.01";
  unsigned base_socket_port = 7000;
  bool verbose = false;
  BoundingBox bb(-1.0f, 1.0f, 0.0f, 2.0f, -1.0f, 1.0f);
  CMDParser p("play_this_filename ...");

  p.addOpt("s",1,"socket_ip", "specify ip address of socket for sending, default: " + socket_ip);
  p.addOpt("p",1,"socket_port", "specify port of socket for sending, default: " + toString(base_socket_port));
  p.addOpt("v",-1,"verbose", "enable output verbosity, default: false");
  p.addOpt("b",6,"bounding_box", "specifies the reconstruction bounding box, default (x_min x_max y_min y_max z_min z_max): -1 1 0 2 -1 1 ");
  p.init(argc,argv);

  if(p.isOptSet("s")){
    socket_ip = p.getOptsString("s")[0];
  }

  if(p.isOptSet("p")){
    base_socket_port = p.getOptsInt("p")[0];
  }

  if(p.isOptSet("v")){
    verbose = true;
  }

  if(p.isOptSet("b")) {
    bb.x_min = p.getOptsFloat("b")[0];
    bb.x_max = p.getOptsFloat("b")[1];
    bb.y_min = p.getOptsFloat("b")[2];
    bb.y_max = p.getOptsFloat("b")[3];
    bb.z_min = p.getOptsFloat("b")[4];
    bb.z_max = p.getOptsFloat("b")[5];
  }

  Meta m;
  m.directory = p.getArgs()[0];
  m.format = "dddd";
  m.start_frame = 50;
  m.end_frame = 200; // 950
  PlyFrameBuffer buffer(m);

  unsigned num_frames = buffer.getFrameCount();
  
  PointCloudEncoder encoder;

  PointCloud<Vec32, Vec32> pc(buffer.getFrame(0)->bounding_box);
  Vec32 pos, clr;
  Vec8 clr_comp;
  for(unsigned idx = 0; idx < buffer.getFrame(0)->position_xyz.size()/3; ++idx) {
    pos.x = buffer.getFrame(0)->position_xyz[3*idx];
    clr_comp.x = buffer.getFrame(0)->color_rgb[3*idx];
    pos.y = buffer.getFrame(0)->position_xyz[3*idx+1];
    clr_comp.y = buffer.getFrame(0)->color_rgb[3*idx+1];
    pos.z = buffer.getFrame(0)->position_xyz[3*idx+2];
    clr_comp.z = buffer.getFrame(0)->color_rgb[3*idx+2];
    clr = encoder.Vec8ToVec32(clr_comp);
    pc.addVoxel(pos, clr);
  }

  zmq::context_t ctx(1); // means single threaded

  const unsigned num_streams = 1;

  std::cout << "going to stream " << num_streams << " to socket ip " << socket_ip << " starting at port number " << base_socket_port << std::endl;

  std::vector<zmq::socket_t* > sockets;
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    zmq::socket_t* socket = new zmq::socket_t(ctx, ZMQ_PUB); // means a publisher
    uint32_t hwm = 1;
    socket->setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
    std::string endpoint("tcp://" + socket_ip + ":" + toString(base_socket_port + s_num));
    socket->bind(endpoint.c_str());
    std::cout << "binding socket to " << endpoint << std::endl;
    sockets.push_back(socket);
  }

  sensor::timevalue ts(sensor::clock::time());
  unsigned f_num = 0;
  while(true){

    pc.clear();

    pc.bounding_box = buffer.getFrame(f_num)->bounding_box;
    for(unsigned idx = 0; idx < buffer.getFrame(f_num)->position_xyz.size()/3; ++idx) {
      pos.x = buffer.getFrame(f_num)->position_xyz[3*idx];
      clr_comp.x = buffer.getFrame(f_num)->color_rgb[3*idx];
      pos.y = buffer.getFrame(f_num)->position_xyz[3*idx+1];
      clr_comp.y = buffer.getFrame(f_num)->color_rgb[3*idx+1];
      pos.z = buffer.getFrame(f_num)->position_xyz[3*idx+2];
      clr_comp.z = buffer.getFrame(f_num)->color_rgb[3*idx+2];
      clr = encoder.Vec8ToVec32(clr_comp);
      pc.addVoxel(pos, clr);
    }

    f_num = (f_num+1) % num_frames;
    if(f_num == 0 && verbose) {
      std::cout << "==================RESTARTING STREAM==================" << std::endl;
    }

    sensor::timevalue start_t(sensor::clock::time());
      
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){

      if(verbose) {
        std::cout << "Sending " << pc.size() << " points.\n";
      }

      // send point cloud
      zmq::message_t msg = encoder.encode(&pc, PointCloudEncoder::PC_3x8p_3x8c);
      sockets[s_num]->send(msg);
    }

    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Reconstruction and sending took " << (end_t - start_t).msec() << "ms.\n";
    }
  }

  // cleanup
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    delete sockets[s_num];
  }
  

  return 0;
}
