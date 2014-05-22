#include "on_top_detector.h"
#include "relationship_manager.h"
#include "contact_points_detector.h"
#include "relationship_manager.h"

void OnTopDetector::computeRelationship()
{
    int obj1_id = recognizedObject1->uniqueId;
    int obj2_id = recognizedObject2->uniqueId;

    bool isOnTop = false;
    int onTopObjectId;

    PropertyManager *pm;
    pm = PropertyManager::getInstance();
    RelationshipManager *rm;
    rm = RelationshipManager::getInstance();
    // check to see proper type casting from pointers to references
    boost::shared_ptr<Relationship> isTouching = rm->getRelationship(recognizedObject1, recognizedObject2, CONTACT_POINTS);
    boost::shared_ptr<ContactPointsRelationship> cp;
    cp = boost::dynamic_pointer_cast<ContactPointsRelationship>(isTouching);

    if(cp->contactPoints.size() > 0)
    {

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject1 = pm->getProperty(recognizedObject1, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com1;
        com1 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject1);

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject2 = pm->getProperty(recognizedObject2, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com2;
        com2 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject2);

        if(com1->centerOfMassPoint.z > com2->centerOfMassPoint.z)
        {
            isOnTop = true;
            detectedRelationship = true;
            onTopObjectId = obj1_id;
            std::cout << "Object "<< obj1_id <<" is on top of Object "<<obj2_id<<std::endl;
        }
        else
        {
            isOnTop = true;
            detectedRelationship = true;
            onTopObjectId = obj2_id;
            std::cout << "Object "<<obj2_id<<" is on top of Object "<<obj1_id<<std::endl;
        }      
    }
    else{
        detectedRelationship = false;
        std::cout << "Objects are not in contact. On top relationship is not defined!"<<std::endl;
    }
        Relationship *onTop = new OnTopRelationship(isOnTop, onTopObjectId);
        computedRelationship = boost::shared_ptr<Relationship>(onTop);
}