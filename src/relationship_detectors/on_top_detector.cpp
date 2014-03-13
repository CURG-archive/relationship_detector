#include "on_top_detector.h"
#include "relationship_manager.h"
#include "relationship_factory.h"
#include "property_manager.h"


void OnTopDetector::calculateRelationshipValue()
{
    int obj1_id = segmentedObject1->uniqueId;
    int obj2_id = segmentedObject2->uniqueId;
    
    if(RelationshipManager::getRelationshipValue(1, obj1_id, obj2_id))
    {
        boost::shared_ptr<CenterOfMassProperty> centerOfMassPropertyObject1 = PropertyManager::getPropertyValue(property_id, obj1_id);
        boost::shared_ptr<CenterOfMassProperty> centerOfMassPropertyObject2 = PropertyManager::getPropertyValue(property_id, obj2_id);
        if(centerOfMassPropertyObject1->centerOfMassPoint.z > centerOfMassPropertyObject2->centerOfMassPoint.z)
        {
            isOnTop = true;
        }
        if(isOnTop)     std::cout << "Object 1 on top of Object 2"<<std::endl;
        else            std::cout << "Object 2 on top of Object 1"<<std::endl;
    
        // as objects are touching one of them has to be on top of other
        onTopRelationship = RelationshipFactory::getInstance().buildOnTopRelationship(segmentedObject1, segmentedObject2, isOnTop);   
    }
    else
    {
        std::cout<<"Objects not touching..."<<std::endl;
    }

}
