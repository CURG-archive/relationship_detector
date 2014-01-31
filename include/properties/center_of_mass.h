#ifndef CENTER_OF_MASS_PROPERTY_H
#define CENTER_OF_MASS_PROPERTY_H

#include "pc_property.h"

class CenterOfMassProperty: public PCProperty
{
public:
	pcl::PointXYZ centerOfMass;

	CenterOfMassProperty():PCProperty(){ propertyName = "CenterOfMass";};	
	~CenterOfMassProperty(){};
}; 

#endif