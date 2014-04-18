#include <property_manager.h>

bool PropertyManager::instanceFlag = false;
PropertyManager* PropertyManager::pm = NULL;

PropertyManager::PropertyManager()
{
  propertyMap.reserve(7000);
  //register the properties to their detectors
  propertyDetectorFactoryMap[PropertyClassIds::CENTER_OF_MASS] = boost::shared_factory<CenterOfMassDetector>();
  // propertyDetectorFactoryMap[PropertyClassIds::COLOR] = boost::shared_factory<ColorPropertyDetector>();    
  // propertyDetectorFactoryMap[PropertyClassIds::ORIENTATION] = boost::shared_factory<OrientationPropertyDetector>();
}

PropertyManager* PropertyManager::getInstance()
{
    if(!instanceFlag)
    {
        pm = new PropertyManager();
        instanceFlag = true;
        return pm;
    }
    else
    {
        return pm;
    }
}

boost::shared_ptr<PCProperty> PropertyManager::getProperty(RecognizedObject *recognizedObject, PropertyType property_type)
{
  int property_hash = pow(2,recognizedObject->uniqueId)*pow(3,property_type);
  if(propertyMap.find(property_hash) != propertyMap.end())
  {
    return propertyMap[property_hash];
  }

  boost::shared_ptr<PropertyDetector> detector = propertyDetectorFactoryMap[property_type]();
  detector->setRecognizedObject(recognizedObject);
  detector->computeProperty();
  propertyMap[property_hash] = detector->getProperty();
  return detector->getProperty();
}

std::vector<boost::shared_ptr<PCProperty>> PropertyManager::getAllProperties(RecognizedObject *recognizedObject)
{
  std::vector<boost::shared_ptr<PCProperty>> allProperties;
  
  for (int i=0; i<NUM_PROPERTIES; i++)
  {
    allProperties.push_back(this->getProperty(recognizedObject, i));
  }
  return allProperties;
}