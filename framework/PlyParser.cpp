#include <PlyParser.hpp>
#include <fstream>
#include <sstream>
#include <cassert>
#include <iostream>

PlyParser::PlyParser()
    : position_xyz()
    , normal_xyz()
    , color_rgb()
    , bounding_box()
    , file_path_()
{}

PlyParser::~PlyParser()
{}

bool PlyParser::parse(const std::string& file_path, bool verbose)
{
    position_xyz.clear();
    normal_xyz.clear();
    color_rgb.clear();

    file_path_ = file_path;

    if(verbose) {
        std::cout << "Parsing .ply file\n";
        std::cout << " > path: " << file_path_ << std::endl;
    }

    std::ifstream infile(file_path_);
    std::string line;
    int i = 0;
    unsigned vertex_count = 0;
    unsigned face_count = 0;
    std::string format = "";
    bool parse_header = true;
    std::istringstream iss;
    unsigned v_idx = 0;
    int percent_done = 0;
    int new_percent_done = 0;
    float x, y, z, nx, ny, nz;
    unsigned short r, g, b;
    while (std::getline(infile, line))
    {
        if(parse_header) {
            if(i == 0) {
                if(line.compare(0, 3, "ply") != 0)
                    return false;
                ++i;
                continue;
            }
            else if(i == 1) {
                if(line.compare(0, 6, "format") != 0)
                    return false;
                format = line;
                format.erase(0, 7);
                ++i;
                continue;
            }
            else if(i == 2) {
                if(line.compare(0, 14, "element vertex") != 0)
                    return false;
                line.erase(0,15);
                iss = std::istringstream(line);
                iss >> vertex_count;
                ++i;
                continue;
            }
            if(!assureProperty(line, "float", "x"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "float", "y"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "float", "z"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "float", "nx"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "float", "ny"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "float", "nz"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "uchar", "diffuse_red"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "uchar", "diffuse_green"))
                return false;
            std::getline(infile, line);
            if(!assureProperty(line, "uchar", "diffuse_blue"))
                return false;
            while(line.compare(0, 10, "end_header") != 0) 
            {
                std::getline(infile, line);
                if(line.compare(0, 12, "element face") == 0) {
                    line.erase(0,13);
                    iss = std::istringstream(line);
                    iss >> face_count;
                }
            }
            parse_header = false;
            position_xyz.resize(vertex_count*3);
            normal_xyz.resize(vertex_count*3);
            color_rgb.resize(vertex_count*3);
            continue;
        }
        
        if(v_idx == vertex_count - 1) {
            if (verbose)
                std::cout << " > DONE.\n";
            return true;
        }
        
        new_percent_done = (unsigned) (((float) v_idx / (float) vertex_count) * 100.0f);

        if(new_percent_done > percent_done) {
            if(verbose)
                std::cout << " > " << new_percent_done << "%\n";
            percent_done = new_percent_done;
        }
        iss = std::istringstream(line);
        iss >> x >> y >> z >> nx >> ny >> nz >> r >> g >> b;
        if(x < bounding_box.x_min)
            bounding_box.x_min = x;
        else if(x > bounding_box.x_max)
            bounding_box.x_max = x;
        if(y < bounding_box.y_min)
            bounding_box.y_min = y;
        else if(y > bounding_box.y_max)
            bounding_box.y_max = y;
        if(z < bounding_box.z_min)
            bounding_box.z_min = z;
        else if(z > bounding_box.z_max)
            bounding_box.z_max = z;
        position_xyz[3*v_idx] = x;
        position_xyz[3*v_idx+1] = y;
        position_xyz[3*v_idx+2] = z;
        normal_xyz[3*v_idx] = nx;
        normal_xyz[3*v_idx+1] = ny;
        normal_xyz[3*v_idx+2] = nz;
        color_rgb[3*v_idx] = r;
        color_rgb[3*v_idx+1] = g;
        color_rgb[3*v_idx+2] = b;
        ++v_idx;
    }
    return false;
}

bool PlyParser::assureProperty(const std::string& line, const std::string& type_name, const std::string& prop_name)
{
    bool failed = false;
    failed = line.compare(0, 8, "property") != 0;
    if(failed)
        return false;
    failed = line.compare(9, type_name.size(), type_name) != 0;
    if(failed)
        return false;
    failed = line.compare(9+type_name.size()+1, prop_name.size(), prop_name) != 0;
    return !failed;
}