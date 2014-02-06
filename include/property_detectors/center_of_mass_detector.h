#include "property_detector.h"
#include "center_of_mass_property.h"
#include "property_factory.h"
#include <boost/shared_ptr.hpp>

class CenterOfMassDetector: public PropertyDetector
{
	protected:

		boost::shared_ptr<CenterOfMassProperty> centerOfMassProperty;

	public:

		virtual void calculatePropertyValue();

		boost::shared_ptr<CenterOfMassProperty> getCenterOfMassProperty();


}; 