

#include "property_factory.h"


boost::shared_ptr<CenterOfMassProperty> PropertyFactory::buildCenterOfMassProperty(int segmentedObjectId, geometry_msgs::Point center_of_mass_point)
{
	int propertyId = computeNextPropertyId();

	return boost::shared_ptr<CenterOfMassProperty>(new CenterOfMassProperty(propertyId, segmentedObjectId, center_of_mass_point));
}
