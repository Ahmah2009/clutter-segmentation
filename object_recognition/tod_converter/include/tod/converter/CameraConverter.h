/*
 * CameraConverter.h
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#ifndef CAMERACONVERTER_H_
#define CAMERACONVERTER_H_

#include <string>

#include <tod/core/Camera.h>

namespace tod
{
  class CameraConverter
  {
  public:
    virtual ~CameraConverter();
    CameraConverter(std::string basePath, std::string resultPath);
    void convert();

    static void readCameraParameters(Camera& cam, const std::string& filename);

  private:
    std::string path;
    std::string resPath;

    const std::string cameraFileName;
    const std::string resultFile;
    const std::string yamlNodeName;

    void writeCameraParameters(const Camera& cam, const std::string& filename);
  };
}

#endif /* CAMERACONVERTER_H_ */
