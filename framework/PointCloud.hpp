#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <DataTypes.hpp>
#include <vector>

struct PointCloud {
	PointCloud()
		: points()
		, colors()
	{}

	~PointCloud() 
	{}

	void clear() 
	{
		points.clear();
		colors.clear();
	}

	void addVoxel(glm::vec3 pos, glm::vec3 clr)
	{
		points.push_back(pos);
		colors.push_back(clr);
	}
	
	unsigned size()
	{
		return points.size();
	}

	std::vector<glm::vec3> points;
	std::vector<glm::vec3> colors;
};

#endif // #ifndef  POINT_CLOUD_HPP