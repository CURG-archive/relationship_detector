#include "relationship.h"

int Relationship::relationship_id_generator = 0;

int Relationship::getNextRelationshipId()
    {
      Relationship::relationship_id_generator +=1;
      return relationship_id_generator;
    }