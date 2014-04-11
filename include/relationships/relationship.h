#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include "recognized_object.h"

class Relationship
{

  public:
  		// //unique id for this property
  		// int relationshipId;

      static int relationship_id_generator;
      int uniqueRelationshipType;

      int uniqueRelationshipId;

      int recognizedObjectId1;
      int recognizedObjectId2;

      int getNextRelationshipId();

      virtual ~Relationship(){};  
};

#endif