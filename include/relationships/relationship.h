#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include "segmented_object.h"

class Relationship
{
  	public:
  		//unique id for this property
  		int relationshipId;

      SegmentedObject *segmentedObject1;
      SegmentedObject *segmentedObject2;


      Relationship(int relationship_id, SegmentedObject *segmented_object_1,SegmentedObject *segmented_object_2)
      {
      	relationshipId = relationship_id;
      	segmentedObject1 = segmented_object_1;
        segmentedObject2 = segmented_object_2;
      };

      virtual ~Relationship(){};
    
};

#endif