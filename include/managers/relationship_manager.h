#ifndef RELATIONSHIP_MANAGER_H
#define RELATIONSHIP_MANAGER_H

#include <ros/package.h>
#include <unordered_map>
#include <map>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/factory.hpp>

#include "relationshipEnum.h"
#include "relationship.h"
#include "recognized_object.h"
#include "on_top_detector.h"
#include "contact_points_detector.h"
#include "relationship_detector.h"
#include "property_manager.h"

using namespace std;
// namespace boost {
//         // Add in a simple wrapper for a factor creating shared pointers
//                 template<typename T>
//         struct shared_factory : public factory<shared_ptr<T> >
//         { };
// }

class RelationshipManager
{
protected:
  typedef boost::function<RelationshipDetectorPtr ()> RelationshipDetectorFactory;
  typedef map<int, RelationshipDetectorFactory> RelationshipDetectorFactoryMap;
  typedef int RelationshipType;

  static const int NUM_RELATIONSHIPS = 1;
  //map of already detected relationships
  //key: hash(object_id and relationship_type)
  //value: relationship (true or false)
  unordered_map<int, boost::shared_ptr<Relationship>> relationshipMap;
  RelationshipDetectorFactoryMap relationshipDetectorFactoryMap;

public:
  //constructor 
  RelationshipManager()
  {
    relationshipMap.reserve(7000);    
    //register the relationships to their detectors
    relationshipDetectorFactoryMap[RelationshipClassIds::CONTACT_POINTS] = boost::shared_factory<ContactPointsDetector>();
    // relationshipDetectorFactoryMap[RelationshipClassIds::ON_TOP] = boost::shared_factory<OnTopDetector>();
  }


  boost::shared_ptr<Relationship> getRelationship(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2, RelationshipType relationship_type)
  {
    int relationship_hash = pow(2,recognizedObject1->uniqueId)*pow(3,recognizedObject2->uniqueId)*pow(5,relationship_type);
    if(relationshipMap.find(relationship_hash) != relationshipMap.end())
    {
      return relationshipMap[relationship_hash];
    }

    boost::shared_ptr<RelationshipDetector> detector = relationshipDetectorFactoryMap[relationship_type]();
    detector->setRecognizedObjects(recognizedObject1,recognizedObject2);
    detector->computeRelationship();
    relationshipMap[relationship_hash] = detector->getRelationship();
    return detector->getRelationship();
  }

  std::vector<boost::shared_ptr<Relationship>> getAllRelationships(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2)
  {
    std::vector<boost::shared_ptr<Relationship>> allRelationships;
    
    for (int i=0; i<NUM_RELATIONSHIPS; i++)
    {
      allRelationships.push_back(this->getRelationship(recognizedObject1,recognizedObject2,i));
    }
    return allRelationships;
  }

};

#endif