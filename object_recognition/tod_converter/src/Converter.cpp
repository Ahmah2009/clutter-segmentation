/*
 * Converter.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: Alexander Shishkov
 */

#include <fstream>
#include <iostream>
#include <tod/converter/Converter.h>
#include <tod/converter/ObjectConverter.h>

using namespace tod;
using namespace std;

Converter::Converter(const string& baseDirectory, const string& resultDirectory): basePath(baseDirectory),
    resultPath(resultDirectory), configName("/config.txt")
{
}

Converter::~Converter()
{
}

void Converter::readConfigFile()
{
 ifstream file;
 string filepath = basePath + configName;
 file.open(filepath.c_str());
 while (!file.eof() && file.good())
 {
   string name;
   int count;
   file >> name >> count;
   if (name.length() == 0)
     continue;
   objectNames.push_back(name);
   imagesCount.push_back(count);
 }
}

void Converter::writeConfigFile(const string& filename, const vector<string>& names)
{
  string content = "";
  for (size_t index = 0; index < names.size(); index++)
  {
    content += names[index] + "\n";
  }

  ofstream configFile;
  configFile.open(filename.c_str());
  configFile << content.substr(0, content.length() - 1);
  configFile.close();
}

void Converter::convert()
{
  readConfigFile();
  for (size_t index = 0; index < objectNames.size(); index++)
  {
    cout << "Reading object " << objectNames[index] << "...";

    ObjectConverter objectConverter(objectNames[index], imagesCount[index], basePath, resultPath);
    objectConverter.convert();

    cout << "done" << endl;
  }
  writeConfigFile(resultPath + "/" + configName, objectNames);
}

