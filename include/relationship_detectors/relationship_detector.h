#include <string>
#include "segmented_object.h"

class RelationshipDetector
{
	protected:

        SegmentedObject *segmentedObject1;
        SegmentedObject *segmentedObject2;

  public:

        RelationshipDetector()
        {
            detectedRelationship = false;
        };

        bool detectedRelationship;
        
    	void setSegmentedObjects( SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2)
        {
           segmentedObject1 = segmented_object_1;
           segmentedObject2 = segmented_object_2;
           detectedRelationship = false;

        };


    	virtual void calculateRelationshipValue()=0;

typedef boost::shared_ptr<RelationshipDetector> RelationshipPtr;

};