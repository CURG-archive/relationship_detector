#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include "segmented_object.h"

class Relationship
{

  public:
  		// //unique id for this property
  		// int relationshipId;

      static int relationship_id_generator;
      int uniqueRelationshipType;

      int uniqueRelationshipId;

      int segmentedObjectId1;
      int segmentedObjectId2;

      int getNextRelationshipId();

      virtual ~Relationship(){};  
};

#endif