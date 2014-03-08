#include <ros/package.h>
#include <string>
#include <iostream>
#include <cmath>

#include "property_manager.h"


PropertyManager::getPropertyValue(int property_id, SegmentedObject *object)
{
  int property_hash = pow(2,object->uniqueId)*pow(3,property_id);
 
  if(propertyTable.find(property_hash) == propertyTable.end()) 
  	{
  		// call methods to generate property
  		if(property_id == 1)
  		{
	  		CenterOfMassDetector centerOfMassDetector = CenterOfMassDetector();
	  		centerOfMassDetector.setSegmentedObject(&object);
	  		centerOfMassDetector.calculatePropertyValue();
	  		boost::shared_ptr<CenterOfMassProperty> centerOfMassProperty = centerOfMassDetector.getCenterOfMassProperty();
  			propertyTable.insert(make_pair(property_hash, centerOfMassProperty->centerOfMassPoint));
  		}
  	}
  return propertyTable[property_hash];
}

