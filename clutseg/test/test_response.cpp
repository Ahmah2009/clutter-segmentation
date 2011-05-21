/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"
#include "clutseg/pose.h"
#include "test.h"

#include <gtest/gtest.h>

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;

struct ResponseFunctionTest : public ::testing::Test {

    void SetUp() {
        samplePose(pose);
        string name = "haltbare_milch";
        np = LabeledPose(name, pose);
        object = new TexturedObject();
        object->name = name;
        groundTruth.labels.push_back(np);
        img_name = "image_00000.png";
        ground[img_name] = groundTruth;
    }

    CutSseResponseFunction sse_response;
    PoseRT pose;
    LabeledPose np;
    Ptr<TexturedObject> object;
    string img_name;
    GroundTruth groundTruth;
    SetGroundTruth ground;
    SetResult result;
    Response resp;

    void expect_sse_response(float expected, PoseRT est_pose) {
        Guess guess(object, poseRtToPose(est_pose), Mat(), Mat(), Mat());
        result.put(img_name, guess);
        sse_response(result, ground, resp);
        EXPECT_NEAR(expected, resp.value, 1e-6);
    }

};

TEST_F(ResponseFunctionTest, CutSseResponseFunctionZero) {
    expect_sse_response(0.0, pose);
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionHalf) {
    expect_sse_response(0.5, rotatePose(pose, randomOrientation(M_PI / 9 / sqrt(2))));
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionQuarter) {
    expect_sse_response(0.25, rotatePose(pose, randomOrientation(M_PI / 18)));
}

TEST_F(ResponseFunctionTest, TestErrorStatistics) {
    // Add second image to ground truth
    string img_name_2 = "image_00001.png";
    PoseRT pose_2 = rotatePose(pose, randomOrientation(M_PI / 4));
    ground[img_name_2].labels.push_back(LabeledPose("haltbare_milch", pose_2));
    
    result.put(img_name, Guess(object, poseRtToPose(pose), Mat(), Mat(), Mat()));
    result.put(img_name_2, Guess(object, poseRtToPose(pose_2), Mat(), Mat(), Mat()));

    sse_response(result, ground, resp); 

    EXPECT_FLOAT_EQ(0, resp.avg_angle_err);
    EXPECT_FLOAT_EQ(0, resp.avg_succ_angle_err);
    EXPECT_FLOAT_EQ(0, resp.avg_trans_err);
    EXPECT_FLOAT_EQ(0, resp.avg_succ_trans_err);
    EXPECT_FLOAT_EQ(0, resp.avg_angle_sq_err);
    EXPECT_FLOAT_EQ(0, resp.avg_succ_angle_sq_err);
    EXPECT_FLOAT_EQ(0, resp.avg_trans_sq_err);
    EXPECT_FLOAT_EQ(0, resp.avg_succ_trans_sq_err);
    EXPECT_FLOAT_EQ(1.0, resp.succ_rate);
    EXPECT_FLOAT_EQ(0, resp.mislabel_rate);
    EXPECT_FLOAT_EQ(0, resp.none_rate);
}

