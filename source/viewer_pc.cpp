#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>
#include <timevalue.hpp>
#include <clock.hpp>
#include <PointCloud.hpp>
#include <PointCloudEncoder.hpp>

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

  PointCloud<Vec32, Vec32> pc;
  PointCloudEncoder encoder;
  while (!win.shouldClose()) {

    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }

    zmq::message_t zmqm;
    
    // receive frames
    sensor::timevalue start_t(sensor::clock::time());
    socket.recv(&zmqm);
    
    encoder.decode(zmqm, &pc);
    
    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Receiving took " << (end_t - start_t).msec() << "ms.\n";
      std::cout << " > Message size: " << zmqm.size() << "\n";
      std::cout << " > Points: " << pc.size() << "\n";
    }

    glPointSize(1.0);
    glBegin(GL_POINTS);

    for(unsigned p_idx = 0; p_idx < pc.size(); ++p_idx) {
      //std::cout << "pos:" << p.x << ", " << p.y << ", " << p.z << std::endl;
      //std::cout << "clr:" << c.x << ", " << c.y << ", " << c.z << std::endl;
      glColor3f(pc.colors[p_idx].x, pc.colors[p_idx].y, pc.colors[p_idx].z);
      glVertex3f(pc.points[p_idx].x, pc.points[p_idx].y, pc.points[p_idx].z);
    }
  
    glEnd();

    win.update();
  }

  return 0;
}

