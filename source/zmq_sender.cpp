#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>

#include <iostream>
#include <vector>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

int main(int argc, char* argv[]){

  glm::vec3 position(0.0,0.0,0.0);
  CMDParser p("socket");
  //p.addOpt("p",3,"position", "specify the 3D position which should be send, default 0.0 0.0 0.0");
  p.init(argc,argv);

  /*if(p.isOptSet("p")){
    position = glm::vec3(p.getOptsFloat("p")[0], p.getOptsFloat("p")[1], p.getOptsFloat("p")[2]);
    std::cout << "setting position to " << position << std::endl;
  }*/

  std::string socket_name(p.getArgs()[0]);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher
#if ZMQ_VERSION_MAJOR < 3
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
#endif 
  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());

  std::vector<glm::vec3> points;
  unsigned num_header_fields = 1;
  unsigned* header = new unsigned[num_header_fields];
  
  const unsigned bytes_header(num_header_fields * sizeof(unsigned));
  unsigned bytes_points;

  unsigned tick = 0;
  unsigned offset = 0;
  while(true){

    points.clear();

    /* initialize random seed: */
    srand (time(NULL));
    // generate random number between 1 and 10
    // of points to be sent
    for(int i = 0; i < rand() % 10 + 1; ++i)
      points.push_back(position);
  
    bytes_points = points.size() * sizeof(glm::vec3);
    header[0] = points.size();

    zmq::message_t zmqm(bytes_header + bytes_points);
    offset = 0;
    memcpy( (unsigned char* ) zmqm.data() + offset, (const unsigned char*) header, bytes_header);
    offset += bytes_header;
    memcpy( (unsigned char* ) zmqm.data() + offset, (const unsigned char*) points.data(), bytes_points);
    socket.send(zmqm);

    ++tick;
  }

  return 0;
}
