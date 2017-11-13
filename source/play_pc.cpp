#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <timevalue.hpp>
#include <clock.hpp>
#include <zmq.hpp>
#include <math.h>

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
  int start_loop = 0;
  int end_loop = 0;
  int num_loops = 0;
  bool perform_loop = false;
  bool swing = false;
  bool rgb_is_compressed = false;
  float max_fps = 20.0;
  std::string socket_ip = "127.0.0.01";
  unsigned base_socket_port = 7000;
  float min_d = 30.0f;
  float max_d = 286.0f;
  bool verbose = false;
  CMDParser p("play_this_filename ...");
  p.addOpt("f",1,"max_fps", "specify how fast in fps the stream should be played, default: 20.0");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");
  p.addOpt("s",1,"socket_ip", "specify ip address of socket for sending, default: " + socket_ip);
  p.addOpt("p",1,"socket_port", "specify port of socket for sending, default: " + toString(base_socket_port));
  p.addOpt("l",2,"loop", "specify a start and end frame for looping, default: " + toString(start_loop) + " " + toString(end_loop));
  p.addOpt("w",-1,"swing", "enable swing looping mode, default: false");
  p.addOpt("n",1,"num_loops", "loop n time, default: loop forever");
  p.addOpt("r",2,"quant_range", "specify min depth and max depth value for quantization range, default (min_d max_d): 30.0 286.0");
  p.addOpt("v",-1,"verbose", "enable output verbosity, default: false");
  p.init(argc,argv);

  if(p.isOptSet("f")){
    max_fps = p.getOptsFloat("f")[0];
  }

  if(p.isOptSet("c")){
    rgb_is_compressed = true;
  }

  if(p.isOptSet("s")){
    socket_ip = p.getOptsString("s")[0];
  }

  if(p.isOptSet("p")){
    base_socket_port = p.getOptsInt("p")[0];
  }

  if(p.isOptSet("l")){
    start_loop = p.getOptsInt("l")[0];
    end_loop = p.getOptsInt("l")[1];
    if(start_loop > end_loop || start_loop < 0 || end_loop < 0){
      std::cerr << "ERROR: -l option must be both positive and second parameter must be greater than first!" << std::endl;
      p.showHelp();
    }
    perform_loop = true;
  }

  if(p.isOptSet("w")){
    swing = true;
  }

  if(p.isOptSet("n")){
    num_loops = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("r")){
    min_d = p.getOptsFloat("r")[0];
    max_d = p.getOptsFloat("r")[1];
    if(min_d > max_d){
      std::cerr << "ERROR: -r option must define min_d < max_d!" << std::endl;
      p.showHelp();
    }
  }

  if(p.isOptSet("v")){
    verbose = true;
  }

  const unsigned num_kinect_cameras(p.getArgs().size() - 1);
  
  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);
  
  RGBDSensor sensor(cfg, num_kinect_cameras-1);
  
  std::vector<CalibVolume*> cvs;
  for(unsigned i = 0; i < num_kinect_cameras; ++i){
    std::string basefilename = p.getArgs()[i];
    std::string filename_xyz(basefilename + "_xyz");
    std::string filename_uv(basefilename + "_uv");
    cvs.push_back(new CalibVolume(filename_xyz.c_str(), filename_uv.c_str()));
  }

  unsigned min_frame_time_ns = 1000000000/max_fps;

  const unsigned colorsize = rgb_is_compressed ? 691200 : 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);
  const size_t frame_size_bytes((colorsize + depthsize) * num_kinect_cameras);
  
  zmq::context_t ctx(1); // means single threaded

  const unsigned num_streams = 1;

  std::cout << "going to stream " << num_streams << " to socket ip " << socket_ip << " starting at port number " << base_socket_port << std::endl;

  std::vector<FileBuffer*> fbs;
  std::vector<zmq::socket_t* > sockets;
  std::vector<int> frame_numbers;
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    FileBuffer* fb = new FileBuffer(p.getArgs()[p.getArgs().size()-1].c_str());
    if(!fb->open("r")){
      std::cerr << "error opening " << p.getArgs()[s_num] << " exiting..." << std::endl;
      return 1;
    }
    else{
      const unsigned n_fs = fb->getFileSizeBytes()/frame_size_bytes;
      std::cout << p.getArgs()[s_num] << " contains " << n_fs << " frames..."  << std::endl;
      if(perform_loop && end_loop > 0 && end_loop > n_fs){
        end_loop = n_fs;
        start_loop = std::min(end_loop, start_loop);
        std::cout << "INFO: setting start loop to " << start_loop << " and end loop to " << end_loop << std::endl;
      }
    }
    fb->setLooping(true);
    fbs.push_back(fb);
    frame_numbers.push_back(0);
    zmq::socket_t* socket = new zmq::socket_t(ctx, ZMQ_PUB); // means a publisher
    uint32_t hwm = 1;
    socket->setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
    std::string endpoint("tcp://" + socket_ip + ":" + toString(base_socket_port + s_num));
    socket->bind(endpoint.c_str());
    std::cout << "binding socket to " << endpoint << std::endl;
    sockets.push_back(socket);
  }

  // compression helper vars
  const unsigned bytes_rgb(colorsize);
  const unsigned bytes_d(depthsize);
  unsigned num_header_fields = 1;
  int* header = new int[num_header_fields];
  const unsigned bytes_header(num_header_fields * sizeof(int));
  unsigned bytes_voxels;

  std::vector<glm::vec3> points;
  std::vector<glm::vec3> colors;
  
  bool fwd = true;
  int loop_num = 1;
  sensor::timevalue ts(sensor::clock::time());
  while(true){

    if(loop_num > num_loops && num_loops > 0){
      break;
    }
    
    sensor::timevalue start_t(sensor::clock::time());
      
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){

      if(perform_loop){
        frame_numbers[s_num] = fwd ? frame_numbers[s_num] + 1 : frame_numbers[s_num] - 1;
        if(frame_numbers[s_num] < start_loop){
          frame_numbers[s_num] = start_loop + 1;
          if(swing){
            fwd = true;
          }

          if(s_num == 0){
            ++loop_num;
          }

        }
        else if(frame_numbers[s_num] > end_loop){
          if(swing){
            fwd = false;
            frame_numbers[s_num] = end_loop - 1;
          }
          else{
            frame_numbers[s_num] = start_loop;
            if(s_num == 0){
              ++loop_num;
            }
          }
        }
        //std::cout << "s_num: " << s_num << " -> frame_number: " << frame_numbers[s_num] << std::endl;
        fbs[s_num]->gotoByte(frame_size_bytes * frame_numbers[s_num]);
      }

      zmq::message_t zmqm(frame_size_bytes);
      fbs[s_num]->read((unsigned char*) zmqm.data(), frame_size_bytes);
      
      // compress depth component and fill zmqm_comp
      unsigned offset = 0;
      for(unsigned int kinect_idx = 0; kinect_idx < num_kinect_cameras; ++kinect_idx) {
        offset = kinect_idx * (bytes_rgb+bytes_d);
        if(kinect_idx == 0) {
          memcpy((unsigned char*) sensor.frame_rgb, (unsigned char*) zmqm.data() + offset, bytes_rgb);
          offset += bytes_rgb;
          memcpy((unsigned char*) sensor.frame_d, (unsigned char*) zmqm.data() + offset, bytes_d);
        }
        else {
          memcpy((unsigned char*) sensor.slave_frames_rgb[kinect_idx-1], (unsigned char*) zmqm.data() + offset, bytes_rgb);
          offset += bytes_rgb;
          memcpy((unsigned char*) sensor.slave_frames_d[kinect_idx-1], (unsigned char*) zmqm.data() + offset, bytes_d);
        }
      }

      points.clear();
      colors.clear();

      // reconstruction
      for(unsigned k_num = 0; k_num < num_kinect_cameras; ++k_num){
        // do 3D recosntruction for each depth pixel
        for(unsigned y = 0; y < sensor.config.size_d.y; ++y){
          for(unsigned x = 0; x < (sensor.config.size_d.x - 3); ++x){
            const unsigned d_idx = y* sensor.config.size_d.x + x;
            float d = k_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[k_num - 1][d_idx];
            if(d < cvs[k_num]->min_d || d > cvs[k_num]->max_d)
              continue;
            
            glm::vec3 pos3D;
            glm::vec2 pos2D_rgb;
            
            pos3D = cvs[k_num]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
                     y * 1.0/sensor.config.size_d.y, d);
            glm::vec2 pos2D_rgb_norm = cvs[k_num]->lookupPos2D_normalized( x * 1.0/sensor.config.size_d.x, 
                           y * 1.0/sensor.config.size_d.y, d);
            pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor.config.size_rgb.x,
                pos2D_rgb_norm.y * sensor.config.size_rgb.y);
            
            glm::vec3 rgb = sensor.get_rgb_bilinear_normalized(pos2D_rgb, k_num);
            
            points.push_back(pos3D);
            colors.push_back(rgb);
          }
        }
      }

      // store num points in header
      header[0] = points.size();
      
      // times 2 because of color
      bytes_voxels = 2 * points.size() * sizeof(glm::vec3);
      
      // package pc into message
      zmq::message_t zmqm_pc(bytes_header + bytes_voxels);

      offset = 0;
      memcpy( (unsigned char* ) zmqm_pc.data() + offset, (const unsigned char*) header, bytes_header);
      offset += bytes_header;
      memcpy( (unsigned char* ) zmqm_pc.data() + offset, (const unsigned char*) points.data(), bytes_voxels / 2);
      offset += bytes_voxels / 2;
      memcpy( (unsigned char* ) zmqm_pc.data() + offset, (const unsigned char*) colors.data(), bytes_voxels / 2);
      
      if(verbose) {
        std::cout << "Sending " << points.size() << " points.\n";
        std::cout << " > message size: " << bytes_header+bytes_voxels << " bytes\n";
      }

      // send point cloud
      sockets[s_num]->send(zmqm_pc);
    }

    sensor::timevalue end_t(sensor::clock::time());
    if(verbose) {
      std::cout << "Reconstruction and sending took " << (end_t - start_t).msec() << "ms.\n";
    }
    // check if fps is correct
    sensor::timevalue ts_now = sensor::clock::time();
    long long time_spent_ns = (ts_now - ts).nsec();
    long long rest_sleep_ns = min_frame_time_ns - time_spent_ns;
    ts = ts_now;
    if(0 < rest_sleep_ns){
      sensor::timevalue rest_sleep(0,rest_sleep_ns);
      nanosleep(rest_sleep);
    }
  }

  // cleanup
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    delete fbs[s_num];
    delete sockets[s_num];
  }

  return 0;
}
