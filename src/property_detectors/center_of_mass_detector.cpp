#include "center_of_mass_detector.h"

CenterOfMassDetector::~CenterOfMassDetector(){}

void CenterOfMassDetector::calculatePropertyValue()
{
    double x_total = 0;
    double y_total = 0;
    double z_total = 0;

    for(int i ; i < objectCloud.size(); i++)
    {
        x_total += objectCloud.points[i].x;
        y_total += objectCloud.points[i].y;
        z_total += objectCloud.points[i].z;
    }

    pcl::PointXYZ center_of_mass_point = pcl::PointXYZ();
    center_of_mass_point.x = x_total/objectCloud.size();
    center_of_mass_point.y = y_total/objectCloud.size();
    center_of_mass_point.z = z_total/objectCloud.size();

    centerOfMassProperty.centerOfMass = center_of_mass_point;
}

CenterOfMassProperty CenterOfMassDetector::getCenterOfMassProperty()
{
	return centerOfMassProperty;
}
