#include <string>
#include "pc_property.h"
#include "segmented_object.h"

class PropertyDetector
{
	//this input segmentedObject
	protected: 
		SegmentedObject *segmentedObject;

	//the property computed from the segmented Object
		boost::shared_ptr<PCProperty> computedProperty;

	public:
	void setSegmentedObject(SegmentedObject *_segmentedObject)
	{
		segmentedObject = _segmentedObject;
	}

	boost::shared_ptr<PCProperty> getProperty()
	{
		return computedProperty;
	}

	//this will be redefined in every base class
	virtual void computeProperty() = 0;
};

typedef boost::shared_ptr<PropertyDetector> PropertyPtr;