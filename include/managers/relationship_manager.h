#ifndef RELATIONSHIP_MANAGER_H
#define RELATIONSHIP_MANAGER_H

#include <ros/package.h>
#include <unordered_map>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/factory.hpp>

#include "relationship.h"
#include "segmented_object.h"
#include "contact_points_detector.h"
#include "relationship_detector.h"

class RelationshipManager
{

  //map of already detected relationships
  //key: hash(object_id and relationship_type)
  //value: relationship (true or false)
  unordered_map<int, Relationship> relationshipMap;

  typedef boost::function<RelationshipPtr (SegmentedObject &segmentedObject1, SegmentedObject &segmentedObject2)> RelationshipDetectorFactory;
  typedef map<int, RelationshipDetectorFactory> RelationshipDetectorFactoryMap;
  typedef int RelationshipType;
  RelationshipDetectorMap RelationshipDetectorFactoryMap;

  //constructor 
  RelationshipManager()
  {
    //register the relationships to their detectors
    relationshipDetectorMap[ContactPointsRelationship.relationship_id] = boost::shared_factory<ContactPointsDetector>();
    // relationshipDetectorMap[OnTopRelationship.relationship_id] = boost::shared_factory<OnTopDetector>();
  }


  Relationship * getRelationship(SegmentedObject * segmentedObject1, SegmentedObject * segmentedObject2, PropertyType relationship_type)
  {
    int relationship_hash = pow(2,segmentedObject1->uniqueId)*pow(3,segmentedObject2->uniqueId)*pow(5,relationship_type);
    if(relationshipMap.find(relationship_hash) != relationshipMap.end())
    {
      return relationshipMap[relationship_hash];
    }

    detector = relationshipDetectorMap[relationship_type]();
    detector.setSegmentedObject(segmentedObject);
    detector.computeRelationship();
    relationshipMap[relationship_hash] = computedRelationship;
    return relationship;
  }

  std::vector<Relationship> getAllRelationships(SegmentedObject *segmentedObject1, SegmentedObject *segmentedObject2)
  {
    std::vector<Relationship> allRelationships;
    
    allRelationships.push_back(this->getRelationship(segmentedObject1,segmentedObject2,1));
    // allRelationships.push_back(this->getRelationship(segmentedObject1,segmentedObject2, 2));
    // allRelationships.push_back(this->getRelationship(segmentedObject1,segmentedObject2, 3));
    return allRelationships;
  }

};

#endif