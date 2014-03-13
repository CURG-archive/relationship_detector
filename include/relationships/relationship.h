#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include "segmented_object.h"

class Relationship
{

  protected:
    static int relationship_id_generator;

  public:
  		// //unique id for this property
  		// int relationshipId;

      int uniqueRelationshipType;

      int uniqueRelationshipId;

      int segmentedObjectId1;
      int segmentedObjectId2;    

      int getNextRelationshipId()
      {
        Relationship::relationship_id_generator +=1;
        return relationship_id_generator;
      }
        // SegmentedObject *segmentedObject1;
        // SegmentedObject *segmentedObject2;


        // Relationship(int relationship_id, SegmentedObject *segmented_object_1,SegmentedObject *segmented_object_2)
        // {
        // 	relationshipId = relationship_id;
        // 	segmentedObject1 = segmented_object_1;
        //   segmentedObject2 = segmented_object_2;
        // };

        virtual ~Relationship(){};  
};

    int Relationship::relationship_id_generator = 0;

#endif