#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>
#include <timevalue.hpp>
#include <clock.hpp>
#include <PointCloud.hpp>

#include <iostream>
#include <fstream>
#include <cassert>
#include <zmq.hpp>

int main(int argc, char* argv[]){

  CMDParser p("basefilename_cv .... serverport");
  p.addOpt("v",-1,"verbose", "enable output verbosity, default: false");
  p.init(argc,argv);

  bool verbose = false;
  if(p.isOptSet("v")){
    verbose = true;
  }

  std::string socket_name(p.getArgs()[0]);

  Window win(glm::ivec2(800,800), true /*3D mode*/);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);;
#if ZMQ_VERSION_MAJOR < 3
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
#endif 
  std::string endpoint("tcp://" + socket_name);
  socket.connect(endpoint.c_str());

  unsigned num_header_fields = 8;
  float* header = new float[num_header_fields];
  const unsigned bytes_header(num_header_fields * sizeof(float));
  unsigned bytes_points;
  unsigned num_points;

  PointCloud* pc = nullptr;
  unsigned offset = 0;
  while (!win.shouldClose()) {

    auto t = win.getTime();
    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }

    zmq::message_t zmqm;
    
    // receive frames
    sensor::timevalue start_t(sensor::clock::time());
    socket.recv(&zmqm);
    
    if(zmqm.size() < bytes_header)
      continue;

    offset = 0;
    memcpy( (unsigned char*) header, (const unsigned char* ) zmqm.data() + offset, bytes_header);
    offset += bytes_header;
    num_points = (unsigned) header[0];
    
    // prepare point cloud
    POINT_CLOUD_TYPE type  = (POINT_CLOUD_TYPE) header[7];
    if(pc==nullptr) {
      pc = PointCloudFactory::createPointCloud(type);
    }
    else if(pc->type() != type) {
      delete pc;
      pc = PointCloudFactory::createPointCloud(type);
    }
    assert(pc != nullptr);

    switch(type) {
      case PC_32: bytes_points = num_points*sizeof(Vec32); break;
      case PC_8: bytes_points = num_points*sizeof(Vec8); break;
      case PC_i32: bytes_points = num_points*sizeof(uint32_t); break;
      default: bytes_points = 0;
    }
    assert(bytes_points != 0);

    pc->clear();
    pc->resize(num_points);
    pc->bounding_box.x_min = header[1];
    pc->bounding_box.x_max = header[2];
    pc->bounding_box.y_min = header[3];
    pc->bounding_box.y_max = header[4];
    pc->bounding_box.z_min = header[5];
    pc->bounding_box.z_max = header[6];

    // copy point cloud data from message 
    memcpy( (unsigned char*) pc->pointsData(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    offset += bytes_points;
    memcpy( (unsigned char*) pc->colorsData(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    
    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Receiving took " << (end_t - start_t).msec() << "ms.\n";
      std::cout << " > Message size: " << zmqm.size() << "\n";
      std::cout << " > Type: " << pc->type() << "\n";
      std::cout << " > Points: " << pc->size() << "\n";
    }

    glPointSize(1.0);
    glBegin(GL_POINTS);

    for(unsigned p_idx = 0; p_idx < pc->size(); ++p_idx) {
      Vec32 c = pc->getColor32(p_idx);
      Vec32 p = pc->getPoint32(p_idx);
      //std::cout << "pos:" << p.x << ", " << p.y << ", " << p.z << std::endl;
      //std::cout << "clr:" << c.x << ", " << c.y << ", " << c.z << std::endl;
      glColor3f(c.x, c.y, c.z);
      glVertex3f(p.x, p.y, p.z);
    }
  
    glEnd();

    win.update();
  }

  return 0;
}

