#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>
#include <timevalue.hpp>
#include <clock.hpp>
#include <PointCloud.hpp>

#include <iostream>
#include <fstream>

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

  unsigned num_header_fields = 7;
  float* header = new float[num_header_fields];
  const unsigned bytes_header(num_header_fields * sizeof(float));
  unsigned bytes_points;
  unsigned num_points;

  //PointCloud pc;
  PointCloud8 pc;
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
    
    //bytes_points = num_points*sizeof(Vec32);
    bytes_points = num_points*sizeof(Vec8);

    pc.clear();
    pc.points.resize(num_points);
    pc.bounding_box.x_min = header[1];
    pc.bounding_box.x_max = header[2];
    pc.bounding_box.y_min = header[3];
    pc.bounding_box.y_max = header[4];
    pc.bounding_box.z_min = header[5];
    pc.bounding_box.z_max = header[6];
    memcpy( (unsigned char*) pc.points.data(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    offset += bytes_points;
    pc.colors.resize(num_points);
    memcpy( (unsigned char*) pc.colors.data(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    
    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Receiving took " << (end_t - start_t).msec() << "ms.\n";
      std::cout << " > Points: " << pc.size() << "\n";
    }

    glPointSize(1.0);
    glBegin(GL_POINTS);

    for(unsigned p_idx = 0; p_idx < pc.size(); ++p_idx) {
      // glColor3f(pc.colors[p_idx].x, pc.colors[p_idx].y, pc.colors[p_idx].z);
      // glVertex3f(pc.points[p_idx].x, pc.points[p_idx].y, pc.points[p_idx].z);
      glColor3f(pc.getColor32(p_idx).x, pc.getColor32(p_idx).y, pc.getColor32(p_idx).z);
      glVertex3f(pc.getPoint32(p_idx).x, pc.getPoint32(p_idx).y, pc.getPoint32(p_idx).z);
    }
  
    glEnd();

    win.update();
  }

  return 0;
}

