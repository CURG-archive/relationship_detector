#include "center_of_mass_detector.h"

void CenterOfMassDetector::calculatePropertyValue()
{
    double x_total = 0;
    double y_total = 0;
    double z_total = 0;

    for(int i =0; i < objectCloudPtr->size() ; i++)
    {
        x_total += objectCloudPtr->points[i].x;
        y_total += objectCloudPtr->points[i].y;
        z_total += objectCloudPtr->points[i].z;
    }

    geometry_msgs::Point center_of_mass_point;
    center_of_mass_point.x = x_total/objectCloudPtr->size();
    center_of_mass_point.y = y_total/objectCloudPtr->size();
    center_of_mass_point.z = z_total/objectCloudPtr->size();

    centerOfMassProperty = PropertyFactory::getInstance().buildCenterOfMassProperty(objectId, center_of_mass_point);
}

boost::shared_ptr<CenterOfMassProperty> CenterOfMassDetector::getCenterOfMassProperty()
{
	return centerOfMassProperty;
}
