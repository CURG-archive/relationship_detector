#include "property_detector.h"

PropertyDetector::PropertyDetector(){};
PropertyDetector::~PropertyDetector(){};
void PropertyDetector::calculatePropertyValue(){};

//The point cloud for the segmented object 
void PropertyDetector::setPointCloud(pcl::PointCloud<pcl::PointXYZ> _objectCloud)
{
	objectCloud = _objectCloud;
}

//returns the name of the property whose value is being calculated
std::string PropertyDetector::getPropertyName()
{
	return propertyName;
}

