#include "on_top_detector.h"
#include "relationship_manager.h"
#include "contact_points_detector.h"
#include "relationship_manager.h"

void OnTopDetector::computeRelationship()
{
    int obj1_id = segmentedObject1->uniqueId;
    int obj2_id = segmentedObject2->uniqueId;

    bool isOnTop = false;
    int onTopObjectId;

    RelationshipManager rm;
    PropertyManager pm;
    // check to see proper type casting from pointers to references
    boost::shared_ptr<Relationship> isTouching = rm.getRelationship(segmentedObject1, segmentedObject2, CONTACT_POINTS);
    boost::shared_ptr<ContactPointsRelationship> cp;
    cp = boost::dynamic_pointer_cast<ContactPointsRelationship>(isTouching);

    if(cp->contactPoints.size() > 0)
    {

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject1 = pm.getProperty(segmentedObject1, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com1;
        com1 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject1);

        boost::shared_ptr<PCProperty> centerOfMassPropertyObject2 = pm.getProperty(segmentedObject2, CENTER_OF_MASS);
        boost::shared_ptr<CenterOfMassProperty> com2;
        com2 = boost::dynamic_pointer_cast<CenterOfMassProperty>(centerOfMassPropertyObject2);

        if(com1->centerOfMassPoint.z > com2->centerOfMassPoint.z)
        {
            isOnTop = true;
            onTopObjectId = obj1_id;
            std::cout << "Object 1 is on top of Object 2"<<std::endl;
        }
        else
        {
            isOnTop = true;
            onTopObjectId = obj2_id;
            std::cout << "Object 2 is on top of Object 1"<<std::endl;
        }      
    }
    else{
        std::cout << "Objects are not in contact. On top relationship is not defined"<<std::endl;
    }
        Relationship *onTop = new OnTopRelationship(isOnTop, onTopObjectId);
        computedRelationship = boost::shared_ptr<Relationship>(onTop);
}