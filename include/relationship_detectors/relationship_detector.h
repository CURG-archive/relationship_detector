#ifndef RELATIONSHIP_DETECTOR_H
#define RELATIONSHIP_DETECTOR_H

#include <string>
#include "segmented_object.h"
#include "relationship.h"


class RelationshipDetector
{
	protected:

        SegmentedObject *segmentedObject1;
        SegmentedObject *segmentedObject2;
      //the property computed from the segmented Object
        boost::shared_ptr<Relationship> computedRelationship;      

  public:
      bool detectedRelationship;

    	void setSegmentedObjects( SegmentedObject *segmented_object_1, SegmentedObject *segmented_object_2)
        {
           segmentedObject1 = segmented_object_1;
           segmentedObject2 = segmented_object_2;
        };

        boost::shared_ptr<Relationship> getRelationship()
        {
          return computedRelationship;
        }
        
        virtual void computeRelationship()=0;
};

typedef boost::shared_ptr<RelationshipDetector> RelationshipDetectorPtr;

#endif