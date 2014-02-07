#include <string>
#include "segmented_object.h"

class PropertyDetector
{
	protected:

    	SegmentedObject *segmentedObject;

  	public:

        PropertyDetector(){};
        
    	void setSegmentedObject(SegmentedObject *segmented_object)
        {
           segmentedObject = segmented_object;
        };


    	virtual void calculatePropertyValue()=0;


};