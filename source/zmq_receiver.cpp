#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>

#include <iostream>
#include <vector>


int main(int argc, char* argv[]){


  CMDParser p("socket");
  p.init(argc,argv);

  std::string socket_name(p.getArgs()[0]);

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

  std::vector<glm::vec3> points;
  unsigned num_header_fields = 1;
  unsigned* header = new unsigned[num_header_fields];  
  
  const unsigned bytes_header(num_header_fields * sizeof(unsigned));
  unsigned num_points;

  unsigned offset = 0;
  while(true){

    //zmq::message_t zmqm(sizeof(glm::vec3));
    zmq::message_t zmqm;
    socket.recv(&zmqm);

    if(zmqm.size() < bytes_header)
      continue;

    offset = 0;
    memcpy( (unsigned char*) header, (const unsigned char* ) zmqm.data() + offset, bytes_header);
    offset += bytes_header;
    num_points = header[0];
    points.clear();
    points.resize(num_points);
    memcpy( (unsigned char*) points.data(), (const unsigned char* ) zmqm.data() + offset, num_points*sizeof(glm::vec3));
    
    std::cout << "received: " << points.size() << " points" << std::endl;

  }

  return 0;
}
