#include <string>
#include <iostream>

#include <cmath>
#include "relationship_manager.h"
#include "contact_points_detector.h"

RelationshipManager::getRelationshipValue(int relationship_id, SegmentedObject *segmentedObject1, SegmentedObject *segmentedObject2)
{
  int relationship_hash = pow(2,segmentedObject1->uniqueId)*pow(3,segmentedObject2->uniqueId)*pow(5,relationship_id);
  if(relationshipTable.find(relationship_hash) == relationshipTable.end())
  {
      if(relationship_id == 1)
      {
          ContactPointsDetector contactPointsDetector = ContactPointsDetector();
          contactPointsDetector.setSegmentedObjects(&segmentedObject1,&segmentedObject2);
          contactPointsDetector.calculateRelationshipValue();
          relationshipTable.insert(make_pair(relationship_hash, contactPointsDetector.detectedRelationship));
      }
  }
  return relationshipTable[relationship_hash];
}