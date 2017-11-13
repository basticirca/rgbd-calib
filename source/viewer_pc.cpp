#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>
#include <timevalue.hpp>
#include <clock.hpp>

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

  unsigned num_header_fields = 1;
  unsigned* header = new unsigned[num_header_fields];
  const unsigned bytes_header(num_header_fields * sizeof(unsigned));
  unsigned bytes_points;
  unsigned num_points;

  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;
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
    num_points = header[0];
    
    bytes_points = num_points*sizeof(glm::vec3);

    points.clear();
    points.resize(num_points);
    memcpy( (unsigned char*) points.data(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    offset += bytes_points;
    colors.clear();
    colors.resize(num_points);
    memcpy( (unsigned char*) colors.data(), (const unsigned char* ) zmqm.data() + offset, bytes_points);
    
    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Receiving took " << (end_t - start_t).msec() << "ms.\n";
      std::cout << " > Points: " << points.size() << "\n";
      std::cout << " > Colors: " << colors.size() << "\n";
    }

    glPointSize(1.0);
    glBegin(GL_POINTS);

    for(unsigned p_idx = 0; p_idx < points.size(); ++p_idx) {
      glColor3f(colors[p_idx].x, colors[p_idx].y, colors[p_idx].z);
      glVertex3f(points[p_idx].x, points[p_idx].y, points[p_idx].z);
    }
  
    glEnd();

    win.update();
  }

  return 0;
}

