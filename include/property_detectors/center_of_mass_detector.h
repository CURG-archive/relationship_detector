#include "property_detector.h"
#include "center_of_mass.h"

class CenterOfMassDetector: public PropertyDetector
{
	protected:

		CenterOfMassProperty centerOfMassProperty;

	public:

		CenterOfMassDetector():PropertyDetector()
			{
				propertyName = "CenterOfMass";
			};
			
		~CenterOfMassDetector();
		void calculatePropertyValue();
		CenterOfMassProperty getCenterOfMassProperty();
}; 