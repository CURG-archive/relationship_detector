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
#include "left_right_adjacency_detector.h"
#include "relationship_detector.h"
#include "property_manager.h"

using namespace std;

class RelationshipManager
{
  private:
  static RelationshipManager *rm;
  static bool instanceFlag;
    //constructor 
    RelationshipManager()
    {
      relationshipMap.reserve(7000);    
      //register the relationships to their detectors
      relationshipDetectorFactoryMap[RelationshipClassIds::CONTACT_POINTS] = boost::shared_factory<ContactPointsDetector>();
      relationshipDetectorFactoryMap[RelationshipClassIds::ON_TOP] = boost::shared_factory<OnTopDetector>();
      relationshipDetectorFactoryMap[RelationshipClassIds::LEFT_RIGHT_ADJACENCY] = boost::shared_factory<LeftRightAdjacencyDetector>();
    }

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
    static RelationshipManager* getInstance();
    boost::shared_ptr<Relationship> getRelationship(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2, RelationshipType relationship_type);
    std::vector<boost::shared_ptr<Relationship>> getAllRelationships(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2);

};

#endif