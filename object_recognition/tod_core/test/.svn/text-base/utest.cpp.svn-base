/*
 * utest.cpp
 *
 *  Created on: Oct 19, 2010
 *      Author: erublee
 */

// Bring in my package's API, which is what I'm testing
#include "tod/core/Features2d.h"
#include "tod/core/Features3d.h"
#include "tod/core/TexturedObject.h"
#include "tod/core/TrainingBase.h"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>
#include <vector>
using namespace cv;
using namespace tod;
using namespace std;
// Declare a test
TEST(Features2dTest, Serialization)
{

  Features2d f2d;
  f2d.keypoints.resize(10);
  f2d.descriptors = Mat::zeros(32, 5, CV_32F);
  Camera camera;
  camera.K = cv::Mat_<float>::eye(3, 3);
  f2d.camera = camera;
  FileStorage fs("f2dtest.yaml", FileStorage::WRITE);
  fs << "features";
  f2d.write(fs);

  fs.release();
  fs = FileStorage("f2dtest.yaml", FileStorage::READ);
  f2d.read(fs["features"]);

}

// Declare a test
TEST(Features3dTest, Serialization)
{

  Features3d f3d;
  Features2d f2d;
  Cloud cloud(20);
  f3d = Features3d(f2d, cloud);
  FileStorage fs("f3dtest.yaml", FileStorage::WRITE);
  fs << "features3d";
  f3d.write(fs);

  fs.release();
  fs = FileStorage("f3dtest.yaml", FileStorage::READ);
  f3d.read(fs["features3d"]);

}
// Declare a test
TEST(TexturedObject, Serialization)
{
  TexturedObject object;
  object.observations.resize(5);

  FileStorage fs("objectTest.yaml", FileStorage::WRITE);
  fs << "object";
  object.write(fs);

  fs.release();
  fs = FileStorage("objectTest.yaml", FileStorage::READ);
  object.read(fs["object"]);

}

// Declare a test
TEST(TrainingBase, Serialization)
{
  TexturedObject object;
  object.observations.resize(5);

  vector<Ptr<TexturedObject> > textured_objects(13);
  for (size_t i = 0; i < textured_objects.size(); i++)
  {
    textured_objects[i] = new TexturedObject(object);
  }

  TrainingBase base(textured_objects);

  FileStorage fs("TrainingBaseTest.yaml", FileStorage::WRITE);
  fs << "training_base";
  base.write(fs);

  fs.release();
  fs = FileStorage("TrainingBaseTest.yaml", FileStorage::READ);
  base.read(fs["training_base"]);

}
namespace
{
struct TCallable
{
  void operator()(const cv::Ptr<TexturedObject>& obj)
  {
    std::cout << obj->id << "\n";
  }
};
}
// Declare a test
TEST(TrainingBase, foreach)
{
  TexturedObject object;
  object.observations.resize(5);

  vector<Ptr<TexturedObject> > textured_objects(13);
  for (size_t i = 0; i < textured_objects.size(); i++)
  {
    textured_objects[i] = new TexturedObject(object);
  }

  TrainingBase base(textured_objects);
  base.forEachObject(TCallable());

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
