#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "center_of_mass.h"

class PropertyDetector
{
	protected:
    	std::string propertyName;
    	pcl::PointCloud<pcl::PointXYZ> objectCloud;

  	public:
        PropertyDetector();
        ~PropertyDetector();

  		//The point cloud for the segmented object 
    	void setPointCloud(pcl::PointCloud<pcl::PointXYZ> _objectCloud);

    	//calculates the property value for the set point cloud
    	virtual void calculatePropertyValue();

    	//returns the name of the property whose value is being calculated
    	std::string getPropertyName();

};