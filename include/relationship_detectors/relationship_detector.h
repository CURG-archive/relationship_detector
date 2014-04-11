#ifndef RELATIONSHIP_DETECTOR_H
#define RELATIONSHIP_DETECTOR_H

#include <string>
#include "recognized_object.h"
#include "relationship.h"


class RelationshipDetector
{
	protected:

        RecognizedObject *recognizedObject1;
        RecognizedObject *recognizedObject2;
      //the property computed from the recognized Object
        boost::shared_ptr<Relationship> computedRelationship;      

  public:
      bool detectedRelationship;

    	void setRecognizedObjects( RecognizedObject *recognized_object_1, RecognizedObject *recognized_object_2)
        {
           recognizedObject1 = recognized_object_1;
           recognizedObject2 = recognized_object_2;
        };

        boost::shared_ptr<Relationship> getRelationship()
        {
          return computedRelationship;
        }
        
        virtual void computeRelationship()=0;
};

typedef boost::shared_ptr<RelationshipDetector> RelationshipDetectorPtr;

#endif