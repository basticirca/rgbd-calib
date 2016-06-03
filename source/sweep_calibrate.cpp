#include <ChessboardSampling.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <CMDParser.hpp>
#include <sweepsampler.hpp>
#include <calibrator.hpp>

#include <fstream>
#include <iostream>
#include <iomanip>


unsigned num_samples_taken = 0;

double sampleQuality(const std::string& filename_xyz, const std::string& filename_uv,
		     ChessboardSampling& cs_sweep,
		     const RGBDConfig& cfg,
		     const Checkerboard& cb,
		     const float tracking_offset_time,
		     const float color_offset_time,
		     const unsigned optimization_stride,
		     const unsigned idwneighbours,
		     bool using_nni,
		     const std::string& basefilename){
    CalibVolume cv_sweep(filename_xyz.c_str(), filename_uv.c_str());
    cs_sweep.loadChessboards();
    SweepSampler ss(&cb, &cv_sweep, &cfg);
    ss.extractSamples(&cs_sweep, tracking_offset_time, color_offset_time, optimization_stride);
    const std::vector<samplePoint>& sps = ss.getSamplePoints();
    Calibrator   c;
    c.using_nni = using_nni;
    c.applySamples(&cv_sweep, sps, cfg, idwneighbours, basefilename.c_str());
    ++num_samples_taken;
    return c.evaluatePlanes(&cv_sweep, &cs_sweep, cfg, optimization_stride);
}

double sampleGradient(const std::string& filename_xyz, const std::string& filename_uv,
		      ChessboardSampling& cs_sweep,
		      const RGBDConfig& cfg,
		      const Checkerboard& cb,
		      const double left_tracking_offset_time,
		      const double right_tracking_offset_time,
		      const float color_offset_time,
		      const unsigned optimization_stride,
		      const unsigned idwneighbours,
		      bool using_nni,
		      const std::string& basefilename){

  const double  y_left = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				       left_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);

  const double  y_right = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					right_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);
  
  return (y_right - y_left)/(right_tracking_offset_time - left_tracking_offset_time);

}

float refine(const std::string& filename_xyz, const std::string& filename_uv,
	     ChessboardSampling& cs_sweep,
	     const RGBDConfig& cfg,
	     const Checkerboard& cb,
	     const float best_tracking_offset_time,
	     const float color_offset_time,
	     const unsigned optimization_stride,
	     const unsigned idwneighbours,
	     bool using_nni,
	     const std::string& basefilename){
  

  const double min_gradient  = 0.000001;
  const double gradient_step = 0.03;
  const double step = 1.0;
  float tracking_offset_time = best_tracking_offset_time;

  std::cout << "refine: starting refinement at "
	    << std::setprecision(10) << tracking_offset_time << std::endl;

  for(unsigned rs = 0; rs < 15; ++rs){
    std::cout << "refine: refining at "
	      << std::setprecision(10) << tracking_offset_time << std::endl;
    double gradient = sampleGradient(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				     tracking_offset_time - 0.5 * gradient_step,
				     tracking_offset_time + 0.5 * gradient_step,
				     color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);
    std::cout << rs << " refine: at " << std::setprecision(10)
	      << tracking_offset_time << " gradient is " << gradient << std::endl;
    if(std::abs(gradient) > min_gradient){
      tracking_offset_time = tracking_offset_time + step * gradient;
    }
    else{
      break;
    }
  }
  
  std::cout << "refine: finishing refinement at " << std::setprecision(10) << tracking_offset_time << std::endl;

  return tracking_offset_time;
}


float optimizeParabelFitting(const std::string& filename_xyz, const std::string& filename_uv,
			     ChessboardSampling& cs_sweep,
			     const RGBDConfig& cfg,
			     const Checkerboard& cb,
			     const float tracking_offset_time_min,
			     const float tracking_offset_time_max,
			     const float color_offset_time,
			     const unsigned optimization_stride,
			     const unsigned idwneighbours,
			     bool using_nni,
			     const std::string& basefilename){

  const double x1 = tracking_offset_time_min;
  const double x2 = 0.5f*(tracking_offset_time_min + tracking_offset_time_max);
  const double x3 = tracking_offset_time_max;

  const double d  = x2 - x1;

  const double  y1 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x1, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);

  const double  y2 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x2, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);

  const double  y3 = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
				   x3, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);

  const double xs = x2 + (0.5*d*(y3 - y1))/(2.0*y2 - y1 - y3);
  return float(xs);
}



float optimizeBruteForce(const std::string& filename_xyz, const std::string& filename_uv,
			 ChessboardSampling& cs_sweep,
			 const RGBDConfig& cfg,
			 const Checkerboard& cb,
			 const float tracking_offset_time_min,
			 const float tracking_offset_time_max,
			 const float tracking_offset_time_step,
			 const float color_offset_time,
			 const unsigned optimization_stride,
			 const unsigned idwneighbours,
			 const std::string& optimize_log,
			 bool using_nni,
			 const std::string& basefilename){

  
  std::ofstream* logfile;
  if(optimize_log != ""){
    logfile = new std::ofstream(optimize_log.c_str());
    *logfile << "tracking_offset_time;avg_quality;best_tracking_offset_time;best_avg_quality" << std::endl;
  }

  float best_tracking_offset_time = (tracking_offset_time_max + tracking_offset_time_min) * 0.5f;
  double best_avg_quality = 0.0f;
  for(float tracking_offset_time = tracking_offset_time_min;
      tracking_offset_time < tracking_offset_time_max;
      tracking_offset_time += tracking_offset_time_step){
    
    const double avg_quality = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
					     tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni, basefilename);


    if(avg_quality > best_avg_quality){
      best_avg_quality = avg_quality;
      best_tracking_offset_time = tracking_offset_time;
    }

    if(logfile != 0){
      *logfile << tracking_offset_time << ";" << std::setprecision(10) << avg_quality << ";" << best_tracking_offset_time << ";" << std::setprecision(10) << best_avg_quality << std::endl;
    }

    std::cout << "tracking_offset_time: " << tracking_offset_time << " (best: " << best_tracking_offset_time << ") -> avg_quality: " << avg_quality << " (best: " << best_avg_quality << ")" << std::endl;

  }
  if(logfile != 0){
    logfile->close();
  }
  return best_tracking_offset_time;
}



int main(int argc, char* argv[]){

  std::string pose_offset_filename = "./poseoffset";
  unsigned cv_width  = 128;
  unsigned cv_height = 128;
  unsigned cv_depth  = 256;
  float    cv_min_d  = 0.5;
  float    cv_max_d  = 4.5;


  // ./sweep_calibrate ../../../data/23.cv ../../../data/23_init ../../../data/23_sweep -t -0.05 -0.0 -o 0 -l ../../../data/23.optimize_log.csv -i

  const float tracking_offset_time_step = 0.001; // in seconds
  float tracking_offset_time_min = -0.05; // in seconds
  float tracking_offset_time_max = 0.0; // in seconds
  float color_offset_time = -0.01;

  unsigned idwneighbours = 20;
  std::string optimize_log("");
  bool using_nni = false;
  const unsigned optimization_stride = 1;
  unsigned optimization_type = 0;
  bool append_samples = false;
  bool undistort = false;
  CMDParser p("calibvolumebasefilename checkerboardview_init checkerboardview_sweep samplesfilename");
  p.addOpt("p",1,"poseoffetfilename", "specify the filename of the poseoffset on disk, default: " + pose_offset_filename);
  p.addOpt("s",3,"size", "use this calibration volume size (width x height x depth), default: 128 128 256");
  p.addOpt("d",2,"depthrange", "use this depth range: 0.5 4.5");

  p.addOpt("t",2,"trackingoffsetrange", "min and max offset in seconds of the tracking system relative to depth frame of the sensor, e.g. -0.05 0.03, default: -0.2 0.0");
  p.addOpt("c",1,"coloroffset", "offset in seconds of the color frame relative to the depth frame of the sensor , e.g. -0.02, default: -0.01");

  p.addOpt("n",1,"numneighbours", "the number of neighbours that should be used for IDW inverse distance weighting, default: 20");

  p.addOpt("o",1,"optimizationtype", "perform optimization using parabel fitting (0), brute force sampling (1), refine with gradient descent (2), midtime is best time (3) , default: 0");

  p.addOpt("l",1,"log", "log the optimization process to given filename, default: no log ");

  p.addOpt("i",-1,"nni", "do use natural neighbor interpolation if possible, default: false");

  p.addOpt("u", -1, "undistort", "enable undistortion of images before chessboardsampling, default: false");

  p.init(argc,argv);

  if(p.getArgs().size() != 4){
    p.showHelp();
  }

  if(p.isOptSet("p")){
    pose_offset_filename = p.getOptsString("p")[0];
    std::cout << "setting poseoffetfilename to " << pose_offset_filename << std::endl;
  }


  if(p.isOptSet("s")){
    cv_width = p.getOptsInt("s")[0];
    cv_height = p.getOptsInt("s")[1];
    cv_depth = p.getOptsInt("s")[2];
  }

  if(p.isOptSet("d")){
    cv_min_d = p.getOptsInt("d")[0];
    cv_max_d = p.getOptsInt("d")[1];
  }


  if(p.isOptSet("t")){
    tracking_offset_time_min = p.getOptsFloat("t")[0];
    tracking_offset_time_max = p.getOptsFloat("t")[1];
  }
  if(p.isOptSet("c")){
    color_offset_time = p.getOptsFloat("c")[0];
  }

  if(p.isOptSet("n")){
    idwneighbours = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("o")){
    optimization_type = p.getOptsInt("o")[0];
  }

  if(p.isOptSet("l")){
    optimize_log = p.getOptsString("l")[0];
  }

  if(p.isOptSet("i")){
    using_nni = true;
  }
  if(p.isOptSet("u")){
    undistort = true;
  }


  const std::string basefilename = p.getArgs()[0];
  std::string filename_xyz(basefilename + "_xyz");
  std::string filename_uv(basefilename + "_uv");
  const std::string filename_yml(basefilename + "_yml");
  CalibVolume cv_init(cv_width, cv_height, cv_depth, cv_min_d, cv_max_d);

  RGBDConfig cfg;
  cfg.read(filename_yml.c_str());

  RGBDSensor sensor(cfg);

  Checkerboard cb;
  cb.load_pose_offset(pose_offset_filename.c_str());
  // configure local 3D Points on chessboard here:
  for(unsigned y = 0; y < CB_HEIGHT; ++y){
    for(unsigned x = 0; x < CB_WIDTH; ++x){
      cb.points_local.push_back(glm::vec3(y * 0.075, x * 0.075,0.0));
    }
  }

  ChessboardSampling cbs(p.getArgs()[1].c_str(), cfg, undistort);
  cbs.init();

  glm::mat4 eye_d_to_world = sensor.guess_eye_d_to_world_static(cbs, cb);
  std::cerr << "extrinsic of sensor is: " << eye_d_to_world << std::endl;
  std::cerr << "PLEASE note, the extrinsic guess can be improved by averaging" << std::endl;

  for(unsigned z = 0; z < cv_init.depth; ++z){
    for(unsigned y = 0; y < cv_init.height; ++y){
      for(unsigned x = 0; x < cv_init.width; ++x){

	const unsigned cv_index = (z * cv_init.width * cv_init.height) + (y * cv_init.width) + x;

	const float depth = (z + 0.5) * (cv_init.max_d - cv_init.min_d)/cv_init.depth + cv_init.min_d;
	const float xd = (x + 0.5) * sensor.config.size_d.x * 1.0/cv_init.width;
	const float yd = (y + 0.5) * sensor.config.size_d.y * 1.0/cv_init.height;

	glm::vec3 pos3D_local = sensor.calc_pos_d(xd, yd, depth);
	glm::vec2 pos2D_rgb   = sensor.calc_pos_rgb(pos3D_local);
	pos2D_rgb.x /= sensor.config.size_rgb.x;
	pos2D_rgb.y /= sensor.config.size_rgb.y;

	glm::vec4 pos3D_world = eye_d_to_world * glm::vec4(pos3D_local.x, pos3D_local.y, pos3D_local.z, 1.0);

	xyz pos3D;
	pos3D.x = pos3D_world.x;
	pos3D.y = pos3D_world.y;
	pos3D.z = pos3D_world.z;
	cv_init.cv_xyz[cv_index] = pos3D;

	uv posUV;
	posUV.u = pos2D_rgb.x;
	posUV.v = pos2D_rgb.y;
	cv_init.cv_uv[cv_index] = posUV;
      }
    }
  }


  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());
  // INITIAL CALIBRATION FINISHED HERE


  // 2. load recording from sweepfilename
  ChessboardSampling cs_sweep(p.getArgs()[2].c_str(), cfg, undistort);
  cs_sweep.init();


  float best_tracking_offset_time = 0.0f;

  switch(optimization_type){
  case 0:
    std::cout << "INFO: performing optimization using parabel fitting." << std::endl;
    best_tracking_offset_time = optimizeParabelFitting(filename_xyz, filename_uv,
						       cs_sweep,
						       cfg,
						       cb,
						       tracking_offset_time_min,
						       tracking_offset_time_max,
						       color_offset_time,
						       optimization_stride,
						       idwneighbours,
						       using_nni,
						       basefilename);
    break;
  case 1:
    std::cout << "INFO: performing optimization using brute force sampling." << std::endl;
    best_tracking_offset_time = optimizeBruteForce(filename_xyz, filename_uv,
						   cs_sweep,
						   cfg,
						   cb,
						   tracking_offset_time_min,
						   tracking_offset_time_max,
						   tracking_offset_time_step,
						   color_offset_time,
						   optimization_stride,
						   idwneighbours,
						   optimize_log,
						   using_nni,
						   basefilename);
    break;
  case 2:
    std::cout << "INFO: performing optimization using refine with gradient descent." << std::endl;
    {
      const float parabel_fit_time = -0.01;/*optimizeParabelFitting(filename_xyz, filename_uv,
							    cs_sweep,
							    cfg,
							    cb,
							    tracking_offset_time_min,
							    tracking_offset_time_max,
							    color_offset_time,
							    optimization_stride,
							    idwneighbours,
							    using_nni,
							    basefilename);*/
      // not needed for release
      const float parabel_fit_value = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
						    parabel_fit_time, color_offset_time, optimization_stride, idwneighbours, using_nni,
						    basefilename);

      best_tracking_offset_time = refine(filename_xyz, filename_uv,
					 cs_sweep,
					 cfg,
					 cb,
					 parabel_fit_time,
					 color_offset_time,
					 optimization_stride,
					 idwneighbours,
					 using_nni,
					 basefilename);
      // not neded for release
      const float refined_value = sampleQuality(filename_xyz, filename_uv, cs_sweep, cfg, cb,
						best_tracking_offset_time, color_offset_time, optimization_stride, idwneighbours, using_nni,
						basefilename);

      std::cout << "num_samples_taken: " << num_samples_taken
		<< " refine finished -> parabel_fit_time: " << std::setprecision(10)
		<< parabel_fit_time
		<< " parabel_fit_value: " << parabel_fit_value
		<< " refined_time: " <<  best_tracking_offset_time
		<< " refined_value: " <<  refined_value
		<< std::endl;

    }
    break;
  case 3:
    best_tracking_offset_time = (tracking_offset_time_min + tracking_offset_time_max) * 0.5f;
    break;
  default:
    std::cerr << "ERROR: invalid optimization_type: " << optimization_type << " -> exiting...!" << std::endl;
    return -1;
    break;
  }

  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  std::cerr << "using best_tracking_offset_time: " << best_tracking_offset_time << std::endl;
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  // do the calibration based on the best_tracking_offset_time
  cs_sweep.loadChessboards();
  SweepSampler ss(&cb, &cv_init, &cfg);
  ss.extractSamples(&cs_sweep, best_tracking_offset_time, color_offset_time);
  ss.appendSamplesToFile(p.getArgs()[3].c_str(), append_samples);
  
  const std::vector<samplePoint>& sps = ss.getSamplePoints();
  Calibrator   c;
  c.using_nni = using_nni;
  c.applySamples(&cv_init, sps, cfg, idwneighbours, basefilename.c_str());


  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());
  filename_xyz += "_sweep";
  filename_uv  += "_sweep";
  cv_init.save(filename_xyz.c_str(), filename_uv.c_str());

  
  return 0;
}
