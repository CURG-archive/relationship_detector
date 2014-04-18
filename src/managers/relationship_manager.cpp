#include <relationship_manager.h>

bool RelationshipManager::instanceFlag = false;
RelationshipManager* RelationshipManager::rm = NULL;

RelationshipManager::RelationshipManager()
{
  relationshipMap.reserve(7000);    
  //register the relationships to their detectors
  relationshipDetectorFactoryMap[RelationshipClassIds::CONTACT_POINTS] = boost::shared_factory<ContactPointsDetector>();
  relationshipDetectorFactoryMap[RelationshipClassIds::ON_TOP] = boost::shared_factory<OnTopDetector>();
  relationshipDetectorFactoryMap[RelationshipClassIds::LEFT_RIGHT_ADJACENCY] = boost::shared_factory<LeftRightAdjacencyDetector>();
}

RelationshipManager* RelationshipManager::getInstance()
{
    if(!instanceFlag)
    {
        rm = new RelationshipManager();
        instanceFlag = true;
        return rm;
    }
    else
    {
        return rm;
    }
}

boost::shared_ptr<Relationship> RelationshipManager::getRelationship(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2, RelationshipType relationship_type)
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

std::vector<boost::shared_ptr<Relationship>> RelationshipManager::getAllRelationships(RecognizedObject *recognizedObject1, RecognizedObject *recognizedObject2)
{
  std::vector<boost::shared_ptr<Relationship>> allRelationships;
  
  for (int i=0; i<NUM_RELATIONSHIPS; i++)
  {
    allRelationships.push_back(this->getRelationship(recognizedObject1,recognizedObject2,i));
  }
  return allRelationships;
}