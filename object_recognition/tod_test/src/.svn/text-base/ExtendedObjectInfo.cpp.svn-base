/*
 * ExtendedObjectInfo.cpp
 *
 *  Created on: Feb 7, 2011
 *      Author: Alexander Shishkov
 */

#include <tod/test/ExtendedObjectInfo.h>
#include <opencv2/core/core.hpp>

using namespace tod;
using namespace cv;

ExtendedObjectInfo::ExtendedObjectInfo()
{
  imageFile = "";
}

ExtendedObjectInfo& ExtendedObjectInfo::operator =(const ExtendedObjectInfo& object_)
{
  if (this != &object_)
  {
    object = object_.object;
    imageFile = object_.imageFile;
  }
  return *this;
}

ExtendedObjectInfo::ExtendedObjectInfo(const ExtendedObjectInfo& object_)
{
  object = object_.object;
  imageFile = object_.imageFile;
}

void ExtendedObjectInfo::loadObjects(cv::FileNode& objectsNode, std::vector<ExtendedObjectInfo>& objects)
{
  objects.clear();

  int objectsCount = 0;
  objectsNode["objectsCount"] >> objectsCount;

  objects.resize(objectsCount);

  for (int i = 0; i < objectsCount; i++)
  {
    stringstream out;
    out << i + 1;
    FileNode object = objectsNode["object" + out.str()];
    object["id"] >> objects[i].object.objectId;
    object["name"] >> objects[i].object.objectName;
    object["rvec"] >> objects[i].object.rvec;
    object["tvec"] >> objects[i].object.tvec;
    object["idx"] >> objects[i].object.imgIdx;
    object["imageName"] >> objects[i].imageFile;
  }
}
