#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <DataTypes.hpp>
#include <vector>
#include <iostream>

// MAPPING HELPER FUNCTIONS

struct Mapper {

    static uint8_t mapTo8Bit(float value, float min, float max)
    {
        if(value < min) {
            return 0;
        }
        else if(value > max) {
            return 255;
        }
        else {
            // map between 0-255;
            float range = max - min;
            value = (value - min) / range * 255;
            value = std::max(0.0f, std::min(value, 255.0f));
            return (uint8_t) value;
        }
    }

    static float mapTo32Bit(uint8_t value, float min, float max, float invalid = 0.0f)
    {
        if(value == 0 || value == 255) {
            return invalid;
        }
        else {
            float range = max - min;
            float res = (float) value;
            res = (res/255.0f)*range + min;
            return res;
        }
    }

};

// uncompressed pointcloud

struct Vec32 {
    Vec32(float x_t=0.0f, float y_t=0.0f, float z_t=0.0f)
        : x(x_t)
        , y(y_t)
        , z(z_t)
    {}

    Vec32(Vec32 const& v)
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {}

    Vec32(glm::vec3& v)
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {}

    ~Vec32()
    {}

    float x;
    float y;
    float z;
};

// BoundBox for pointcloud sample ranges

struct BoundingBox {
    BoundingBox(float x_min_t=.0f, float x_max_t=.0f, float y_min_t=.0f, float y_max_t=.0f, float z_min_t=.0f, float z_max_t=.0f)
        : x_min(x_min_t)
        , x_max(x_max_t)
        , y_min(y_min_t)
        , y_max(y_max_t)
        , z_min(z_min_t)
        , z_max(z_max_t)
    {}

    BoundingBox(BoundingBox const& bb)
        : x_min(bb.x_min)
        , x_max(bb.x_max)
        , y_min(bb.y_min)
        , y_max(bb.y_max)
        , z_min(bb.z_min)
        , z_max(bb.z_max)
    {}

    ~BoundingBox()
    {}

    bool contains(Vec32 const& v) 
    {
        return v.x > x_min && v.x < x_max &&
            v.y > y_min && v.y < y_max &&
            v.z > z_min && v.z < z_max;
    }

    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};


struct PointCloud {
    PointCloud()
        : points()
        , colors()
        , bounding_box()
    {}

    ~PointCloud() 
    {}

    void clear() 
    {
        points.clear();
        colors.clear();
    }

    void addVoxel(Vec32 const& pos, Vec32 const& clr)
    {
        if(!bounding_box.contains(pos))
            return;
        points.push_back(pos);
        colors.push_back(clr);
    }
    
    unsigned size()
    {
        return points.size();
    }

    std::vector<Vec32> points;
    std::vector<Vec32> colors;
    BoundingBox bounding_box;
};

// 8 bit pointcloud

struct Vec8 {
    Vec8(uint8_t x_t=0, uint8_t y_t=0, uint8_t z_t=0)
        : x(x_t)
        , y(y_t)
        , z(z_t)
    {}

    Vec8(Vec8 const& v)
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {}

    Vec8(Vec32 const& v_32, BoundingBox const& bb)
        : x(Mapper::mapTo8Bit(v_32.x, bb.x_min, bb.x_max))
        , y(Mapper::mapTo8Bit(v_32.y, bb.y_min, bb.y_max))
        , z(Mapper::mapTo8Bit(v_32.z, bb.z_min, bb.z_max))
    {}

    ~Vec8()
    {}

    uint8_t x;
    uint8_t y;
    uint8_t z;
};

struct PointCloud8 {
    PointCloud8()
        : points()
        , colors()
        , bounding_box()
        , color_bounds(0.0f,1.0f,0.0f,1.0f,0.0f,1.0f)
    {}

    ~PointCloud8() 
    {}

    void clear() 
    {
        points.clear();
        colors.clear();
    }

    void addVoxel(Vec32 const& pos, Vec32 const& clr)
    {
        if(!bounding_box.contains(pos))
            return;
        points.push_back(Vec8(pos, bounding_box));
        colors.push_back(Vec8(clr, color_bounds));
    }

    void addVoxel(Vec8 const& pos, Vec8 const& clr)
    {
        points.push_back(pos);
        colors.push_back(clr);
    }

    Vec32 const getColor32(unsigned idx)
    {
        Vec32 res;
        if(idx >= size())
            return res;
        res.x = Mapper::mapTo32Bit(colors[idx].x, color_bounds.x_min, color_bounds.x_max);
        res.y = Mapper::mapTo32Bit(colors[idx].y, color_bounds.y_min, color_bounds.y_max);
        res.z = Mapper::mapTo32Bit(colors[idx].z, color_bounds.z_min, color_bounds.z_max);
        return res;
    }

    Vec32 const getPoint32(unsigned idx)
    {
        Vec32 res;
        if(idx >= size())
            return res;
        res.x = Mapper::mapTo32Bit(points[idx].x, bounding_box.x_min, bounding_box.x_max);
        res.y = Mapper::mapTo32Bit(points[idx].y, bounding_box.y_min, bounding_box.y_max);
        res.z = Mapper::mapTo32Bit(points[idx].z, bounding_box.z_min, bounding_box.z_max);
        return res;
    }
    
    unsigned size()
    {
        return points.size();
    }

    std::vector<Vec8> points;
    std::vector<Vec8> colors;
    BoundingBox bounding_box;
    BoundingBox color_bounds;
};

#endif // #ifndef  POINT_CLOUD_HPP