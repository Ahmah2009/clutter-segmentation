/*
 * ExtendedObjectInfo.h
 *
 *  Created on: Feb 7, 2011
 *      Author: Alexander Shishkov
 */

#ifndef EXTENDEDOBJECTINFO_H_
#define EXTENDEDOBJECTINFO_H_

#include <tod/detecting/Recognizer.h>

namespace tod
{
  struct ExtendedObjectInfo
  {
  public:
    ObjectInfo object;
    string imageFile;

    ExtendedObjectInfo();
    ExtendedObjectInfo & operator =(const ExtendedObjectInfo& object_);
    ExtendedObjectInfo(const ExtendedObjectInfo& object_);
    static void loadObjects(cv::FileNode& objectsNode, std::vector<ExtendedObjectInfo>& objects);
  };
}


#endif /* EXTENDEDOBJECTINFO_H_ */
