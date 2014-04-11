#ifndef PROPERTY_DETECTOR_H
#define PROPERTY_DETECTOR_H

#include <string>
#include "pc_property.h"
#include "recognized_object.h"

class PropertyDetector
{
	//this input recognizedObject
	protected: 
		RecognizedObject *recognizedObject;

	//the property computed from the recognized Object
		boost::shared_ptr<PCProperty> computedProperty;

	public:
	void setRecognizedObject(RecognizedObject *_recognizedObject)
	{
		recognizedObject = _recognizedObject;
	}

	boost::shared_ptr<PCProperty> getProperty()
	{
		return computedProperty;
	}

	//this will be redefined in every base class
	virtual void computeProperty() = 0;
};

typedef boost::shared_ptr<PropertyDetector> PropertyDetectorPtr;
#endif