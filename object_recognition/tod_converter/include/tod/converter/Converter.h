/*
 * Converter.h
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <string>
#include <vector>

namespace tod
{
  class Converter
  {
  public:
    virtual ~Converter();
    Converter(const std::string& baseDirectory, const std::string& resultDirectory);

    void convert();
  private:
    std::string basePath;
    std::string resultPath;
    std::vector<std::string> objectNames;
    std::vector<int> imagesCount;

    const std::string configName;

    void readConfigFile();
    void writeConfigFile(const std::string& filename, const std::vector<std::string>& names);
  };
}
#endif /* CONVERTER_H_ */
