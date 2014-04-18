#include "left_right_adjacency_detector.h"
#include "relationship_manager.h"
#include "contact_points_detector.h"
#include "relationship_manager.h"

void LeftRightAdjacencyDetector::computeRelationship()
{
    int obj1_id = recognizedObject1->uniqueId;
    int obj2_id = recognizedObject2->uniqueId;

    bool isLeftRightAdjacent = false;
    int leftObjectId, rightObjectId;
    PropertyManager *pm;
    pm = PropertyManager::getInstance();
    RelationshipManager *rm;
    rm = RelationshipManager::getInstance();
    // PropertyManager pm;
    // check to see proper type casting from pointers to references
    boost::shared_ptr<Relationship> isTouching = rm->getRelationship(recognizedObject1, recognizedObject2, CONTACT_POINTS);
    boost::shared_ptr<ContactPointsRelationship> cp;
    cp = boost::dynamic_pointer_cast<ContactPointsRelationship>(isTouching);

    if(!cp->isTouching)        
    {

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject1 = pm->getProperty(recognizedObject1, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com1;
        com1 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject1);

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject2 = pm->getProperty(recognizedObject2, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com2;
        com2 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject2);

        // if the two point clouds are not touching
        // and if the center of mass of the first point lcoud is to the left of second point cloud
        if(com1->centerOfMassPoint.x < com2->centerOfMassPoint.x)
        {
            isLeftRightAdjacent = true;
            leftObjectId = obj1_id;
            rightObjectId = obj2_id;
            detectedRelationship = true;
            std::cout << "Object 1 is left adjacent to Object 2"<<std::endl;
        }
        else
        {
            isLeftRightAdjacent = true;
            leftObjectId = obj2_id;
            rightObjectId = obj1_id;
            detectedRelationship = true;
            std::cout << "Object 2 is right adjacent to Object 1"<<std::endl;
        }      
    }
    // if the two objects are touching then we are unable to segment them
    // so no relationships detected
    else{
        detectedRelationship = false;
        std::cout << "Objects in contact. Segmentation not possible. Adjacency relationship not defined."<<std::endl;
    }
        Relationship *leftRightAdjacency = new LeftRightAdjacencyRelationship(isLeftRightAdjacent, leftObjectId, rightObjectId);
        computedRelationship = boost::shared_ptr<Relationship>(leftRightAdjacency);
}