#ifndef PLY_PARSER_HPP
#define PLY_PARSER_HPP

#include <string>
#include <vector>
#include <DataTypes.hpp>
#include <PlyParser.hpp> // BoundingBox
#include <PointCloud.hpp>


class PlyParser {

public:
    PlyParser();
    ~PlyParser();

    bool parse(const std::string& file_path, bool verbose = false);

    std::vector<float> position_xyz;
    std::vector<float> normal_xyz;
    std::vector<unsigned short> color_rgb;
    BoundingBox bounding_box;

private:
    bool assureProperty(const std::string& line, const std::string& type_name, const std::string& prop_name);
    
    std::string file_path_;
};

#endif  // #ifndef PLY_PARSER_HPP