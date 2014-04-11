#include "center_of_mass_detector.h"

void CenterOfMassDetector::computeProperty()
{
    double x_total = 0;
    double y_total = 0;
    double z_total = 0;

    for(int i =0; i < recognizedObject->pointCloudPtr->size() ; i++)
    {
        x_total += recognizedObject->pointCloudPtr->points[i].x;
        y_total += recognizedObject->pointCloudPtr->points[i].y;
        z_total += recognizedObject->pointCloudPtr->points[i].z;
    }

    geometry_msgs::Point center_of_mass_point;
    center_of_mass_point.x = x_total/recognizedObject->pointCloudPtr->size();
    center_of_mass_point.y = y_total/recognizedObject->pointCloudPtr->size();
    center_of_mass_point.z = z_total/recognizedObject->pointCloudPtr->size();

    PCProperty *p = new CenterOfMassProperty(center_of_mass_point);
    computedProperty = boost::shared_ptr<PCProperty>(p);
}