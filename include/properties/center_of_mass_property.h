#ifndef CENTER_OF_MASS_PROPERTY_H
#define CENTER_OF_MASS_PROPERTY_H

#include "pc_property.h"
#include <geometry_msgs/Point.h>

class CenterOfMassProperty: public PCProperty
{
	public:
		geometry_msgs::Point centerOfMassPoint;

		CenterOfMassProperty(int property_id, int segmented_object_id, geometry_msgs::Point center_of_mass_point)
			:PCProperty(property_id, segmented_object_id)
		{
			centerOfMassPoint = center_of_mass_point;
		};	

}; 

#endif