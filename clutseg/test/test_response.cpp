/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"
#include "clutseg/pose.h"
#include "test.h"

#include <cv.h>
#include <gtest/gtest.h>
#include <vector>

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;


struct ResponseFunctionTest : public ::testing::Test {

    void SetUp() {
    }

    CutSseResponseFunction sse_response;

};
TEST_F(ResponseFunctionTest, CutSseResponseFunctionZero) {
    string name = "haltbare_milch";
    PoseRT pose;
    samplePose(pose);
    LabeledPose np(name, pose);
    Ptr<TexturedObject> object = new TexturedObject();
    object->name = name;
    Guess guess(object, poseRtToPose(pose), Mat(), Mat(), Mat());
    GroundTruth groundTruth;
    groundTruth.labels.push_back(np);
    SetGroundTruth ground;
    ground["image_00000.png"] = groundTruth;
    SetResult result;
    result.put("image_00000.png", guess);
    Response response;
    sse_response(result, ground, response);
    EXPECT_NEAR(0.0, response.value, 1e-6);
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionHalf) {
    string name = "haltbare_milch";
    PoseRT pose;
    samplePose(pose);
    LabeledPose np(name, pose);
    Ptr<TexturedObject> object = new TexturedObject();
    object->name = name;
    PoseRT est_pose = rotatePose(pose, randomOrientation(M_PI / 9 / sqrt(2)));
    Guess guess(object, poseRtToPose(est_pose), Mat(), Mat(), Mat());
    GroundTruth groundTruth;
    groundTruth.labels.push_back(np);
    SetGroundTruth ground;
    ground["image_00000.png"] = groundTruth;
    SetResult result;
    result.put("image_00000.png", guess);
    Response response;
    sse_response(result, ground, response);
    EXPECT_NEAR(0.5, response.value, 1e-6);
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionQuarter) {
    string name = "haltbare_milch";
    PoseRT pose;
    samplePose(pose);
    LabeledPose np(name, pose);
    Ptr<TexturedObject> object = new TexturedObject();
    object->name = name;
    PoseRT est_pose = rotatePose(pose, randomOrientation(M_PI / 18));
    Guess guess(object, poseRtToPose(est_pose), Mat(), Mat(), Mat());
    GroundTruth groundTruth;
    groundTruth.labels.push_back(np);
    SetGroundTruth ground;
    ground["image_00000.png"] = groundTruth;
    SetResult result;
    result.put("image_00000.png", guess);
    Response response;
    sse_response(result, ground, response);
    EXPECT_NEAR(0.25, response.value, 1e-6);
}
