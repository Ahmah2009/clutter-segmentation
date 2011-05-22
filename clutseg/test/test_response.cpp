/**
 * Author: Julius Adorf
 */

#include "clutseg/response.h"
#include "clutseg/sipc.h"
#include "clutseg/pose.h"
#include "test.h"

#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <limits>

using namespace clutseg;
using namespace cv;
using namespace opencv_candidate;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

struct ResponseFunctionTest : public ::testing::Test {

    // We need to construct a result of an experiment i.e. an instance of
    // SetResult and compare that to the ground truth (i.e. SetGroundTruth).
    // Then we can verify that the statistics work as expected. For covering
    // several cases, we might include
    // - an empty scene
    // - a non-empty scene with no choice being made
    // - a non-empty scene with a false positive at close to an object
    // - a non-empty scene with a false positive far away from any object
    // - a non-empty scene with a true positive exceeding max_trans_error
    // - a non-empty scene with a true positive exceeding max_angle_error
    // - a non-empty scene with a true positive within max_trans_error and max_angle_error
    // - ...
    // For simplification, we can use the same scene with different mocked
    // choices. The choices are made in such a way that we can infer bounds on
    // the generated statistics, or even check for equality.

    bfs::path p;

    // Take two different scenes ...
    GroundTruth empty_scene;
    GroundTruth at_hm_jc;

    // ... and mock up nine different tests. We abbreviate assam_tea by 'at',
    // icedtea by 'it', haltbare_milch by 'hm' and jacobs_coffee by 'jc'. All
    // guesses and their names indicate their characteristics relative to
    // ground truth at_hm_jc.
    Guess at_perfect;
    Guess at_ge_max_angle;
    Guess at_ge_max_trans;
    Guess at_ge_max_trans_angle;
    Guess it_ge_max_angle;
    Guess it_ge_max_trans;
    Guess at_close;
    Guess it_close;
    Guess at_max_trans;
    Guess at_max_angle;
    Guess at_max_trans_angle;

    Guess createGuess(const string & subject, const PoseRT & pose) {
        Ptr<TexturedObject> s = new TexturedObject();
        s->name = subject;
        return Guess(s, poseRtToPose(pose), Mat(), Mat(), Mat());
    }

    void SetUp() {
        p = bfs::path(getenv("CLUTSEG_PATH"));
        {
            empty_scene = GroundTruth();
            at_hm_jc.read(p / "ias_kinect_test_grounded_21/assam_tea_-15_haltbare_milch_0_jacobs_coffee_13/image_00008.png.ground.yaml");
            Pose atp = poseRtToPose(at_hm_jc.posesOf("assam_tea")[0]);
            at_perfect = createGuess("assam_tea", atp);
            at_ge_max_angle = createGuess("assam_tea", rotatePose(atp, randomOrientation(M_PI / 8)));
            at_ge_max_trans = createGuess("assam_tea", translatePose(atp, (Mat_<double>(3, 1) << 0.02, 0.03, 0.01)));
            at_ge_max_trans_angle = createGuess("assam_tea", translatePose(rotatePose(atp, randomOrientation(M_PI / 8)), (Mat_<double>(3, 1) << 0.02, 0.02, 0.03)));
            it_ge_max_angle = createGuess("icedtea", rotatePose(atp, randomOrientation(M_PI / 8)));
            it_ge_max_trans = createGuess("icedtea", translatePose(atp, (Mat_<double>(3, 1) << 0.02, 0.03, 0.01)));
            at_close = createGuess("assam_tea", translatePose(rotatePose(atp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
            it_close = createGuess("icedtea", translatePose(rotatePose(atp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
            at_max_angle = createGuess("assam_tea", rotatePose(atp, randomOrientation(M_PI / 9)));
            at_max_trans = createGuess("assam_tea", translatePose(atp, (Mat_<double>(3, 1)  << 0.02, 0.02, 0.008)));
            at_max_trans_angle = createGuess("assam_tea", translatePose(rotatePose(atp, randomOrientation(M_PI / 9 - 0.02)), (Mat_<double>(3, 1) << 0.02, 0.02, 0.008)));
        }
        
        {
            samplePose(pose);
            string name = "haltbare_milch";
            object = new TexturedObject();
            object->name = name;
            img_name_single = "image_00000.png";
            ground_single[img_name_single].labels.push_back(LabeledPose(name, pose));
        }
    }

    CutSseResponseFunction sse_response_function;
    PoseRT pose;
    Ptr<TexturedObject> object;
    string img_name_single;
    SetGroundTruth ground_single;
    Response rsp;

    void expect_sse_response_single(float expected, PoseRT est_pose) {
        Guess guess(object, poseRtToPose(est_pose), Mat(), Mat(), Mat());
        SetResult result_single;
        result_single.put(img_name_single, guess);
        sse_response_function(result_single, ground_single, rsp);
        EXPECT_NEAR(expected, rsp.value, 1e-6);
    }

};

TEST_F(ResponseFunctionTest, CutSseResponseFunctionZero) {
    expect_sse_response_single(0.0, pose);
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionHalf) {
    expect_sse_response_single(0.5, rotatePose(pose, randomOrientation(M_PI / 9 / sqrt(2))));
}

TEST_F(ResponseFunctionTest, CutSseResponseFunctionQuarter) {
    expect_sse_response_single(0.25, rotatePose(pose, randomOrientation(M_PI / 18)));
}

TEST_F(ResponseFunctionTest, TestErrorStatistics) {
    // Add second image to ground truth
    string img_name_2 = "image_00001.png";
    PoseRT pose_2 = rotatePose(pose, randomOrientation(M_PI / 4));
    SetGroundTruth ground_double = ground_single;
    ground_double[img_name_2].labels.push_back(LabeledPose("haltbare_milch", pose_2));

    SetResult result;
    result.put(img_name_single, Guess(object, poseRtToPose(pose), Mat(), Mat(), Mat()));
    result.put(img_name_2, Guess(object, poseRtToPose(pose_2), Mat(), Mat(), Mat()));

    sse_response_function(result, ground_double, rsp); 

    EXPECT_NEAR(0, rsp.avg_angle_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_angle_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_trans_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_trans_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_angle_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_trans_sq_err, 1e-6);
    EXPECT_NEAR(1.0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
}

TEST_F(ResponseFunctionTest, PerfectEstimatesOnly) {
    SetResult r;
    r.put("at_hm_jc_1", at_perfect);
    r.put("at_hm_jc_2", at_perfect);
    r.put("at_hm_jc_3", at_perfect);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, rsp);

    EXPECT_NEAR(0, rsp.avg_angle_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_angle_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_trans_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_trans_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_angle_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_NEAR(0, rsp.avg_succ_trans_sq_err, 1e-6);
    EXPECT_NEAR(1.0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
}

TEST_F(ResponseFunctionTest, BadEstimatesOnly) {
    SetResult r;
    r.put("at_hm_jc_1", at_ge_max_angle);
    r.put("at_hm_jc_2", at_ge_max_trans);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, rsp);

    EXPECT_NEAR(M_PI / 16, rsp.avg_angle_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_err));
    EXPECT_NEAR(sqrt(0.0014) / 2, rsp.avg_trans_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_err));
    EXPECT_NEAR(M_PI * M_PI / 128, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_sq_err));
    EXPECT_NEAR(0.0007, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_sq_err));
    EXPECT_NEAR(0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
    // Correct labels produced, and half score for poses, so get 75% of points
    // in total. 
    EXPECT_NEAR(1.5, rsp.sipc_score.final_score, 1e-6);
    EXPECT_NEAR(0.75, rsp.sipc_score.final_grade, 1e-6);
}

TEST_F(ResponseFunctionTest, ReallyBadEstimatesOnly) {
    SetResult r;
    r.put("at_hm_jc_1", at_ge_max_trans_angle);
    r.put("at_hm_jc_2", at_ge_max_trans_angle);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, rsp);

    float a = angle_between(at_ge_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    float t = dist_between(at_ge_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    EXPECT_NEAR(a, rsp.avg_angle_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_err));
    EXPECT_NEAR(t, rsp.avg_trans_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_err));
    EXPECT_NEAR(a*a, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_sq_err));
    EXPECT_NEAR(t*t, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_sq_err));
    EXPECT_NEAR(0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
    // Correct labels produced, but no score for poses, so get only half of the
    // points in total. 
    EXPECT_NEAR(1, rsp.sipc_score.final_score, 1e-6);
    EXPECT_NEAR(0.5, rsp.sipc_score.final_grade, 1e-6);
}


TEST_F(ResponseFunctionTest, NonesOnly) {
    SetResult r;
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, rsp);

    EXPECT_TRUE(isnan(rsp.avg_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_sq_err));
    EXPECT_NEAR(0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(1, rsp.none_rate, 1e-6);   
    rsp.sipc_score.print();
}

/** Consider a scene where the recognizer does not make any choice at all.
 * We cannot update any error statistics that involve angle and translation error.
 * We may not include these scenes when averaging later, otherwise we could improve
 * on these error statistics by return more NONEs.
 */
TEST_F(ResponseFunctionTest, NonesDoNotPullDownAverage) {
    SetResult r;
    r.put("at_hm_jc_1", at_max_trans_angle);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, rsp);
    float a = angle_between(at_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    float t = dist_between(at_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    EXPECT_NEAR(a, rsp.avg_angle_err, 1e-6);
    EXPECT_NEAR(a, rsp.avg_succ_angle_err, 1e-6);
    EXPECT_NEAR(t, rsp.avg_trans_err, 1e-6);
    EXPECT_NEAR(t, rsp.avg_succ_trans_err, 1e-6);
    EXPECT_NEAR(a*a, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_NEAR(a*a, rsp.avg_succ_angle_sq_err, 1e-6);
    EXPECT_NEAR(t*t, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_NEAR(t*t, rsp.avg_succ_trans_sq_err, 1e-6);
    EXPECT_NEAR(1./3, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(2./3, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
}

/** Handling of the case in which an object has been labeled that is not even
 * on the scene. */
TEST_F(ResponseFunctionTest, MislabelingsOnly) {
    SetResult r;
    r.put("at_hm_jc_1", it_close);
    r.put("at_hm_jc_2", it_ge_max_angle);
    r.put("at_hm_jc_3", it_ge_max_trans);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, rsp);
    EXPECT_TRUE(isnan(rsp.avg_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_sq_err));
    EXPECT_NEAR(0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(1, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    rsp.sipc_score.print();
}

 /* Realistic example with all kinds of cases occuring. */
TEST_F(ResponseFunctionTest, PerfectNoneMislabelSuccessFail) {
    SetResult r;
    r.put("at_hm_jc_1", at_perfect);
    r.put("at_hm_jc_3", it_close);
    r.put("at_hm_jc_4", at_close);
    r.put("at_hm_jc_5", at_ge_max_trans_angle);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    g["at_hm_jc_4"] = at_hm_jc;
    g["at_hm_jc_5"] = at_hm_jc;
    sse_response_function(r, g, rsp);
    float a = angle_between(at_close.aligned_pose(), at_perfect.aligned_pose());
    float t = dist_between(at_close.aligned_pose(), at_perfect.aligned_pose());
    float fa = angle_between(at_ge_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    float ft = dist_between(at_ge_max_trans_angle.aligned_pose(), at_perfect.aligned_pose());
    EXPECT_NEAR(a, M_PI / 18, 1e-6);
    EXPECT_NEAR(fa, M_PI / 8, 1e-6);
    EXPECT_NEAR((0 + a + fa) / 3, rsp.avg_angle_err, 1e-6);
    EXPECT_NEAR((0 + a*a + fa*fa) / 3, rsp.avg_angle_sq_err, 1e-6);
    EXPECT_NEAR((0 + t + ft) / 3, rsp.avg_trans_err, 1e-6);
    EXPECT_NEAR((0 + t*t + ft*ft) / 3, rsp.avg_trans_sq_err, 1e-6);
    EXPECT_NEAR((0 + t) / 2, rsp.avg_succ_trans_err, 1e-6);
    EXPECT_NEAR((0 + t*t) / 2, rsp.avg_succ_trans_sq_err, 1e-6);
    EXPECT_NEAR((0 + a) / 2, rsp.avg_succ_angle_err, 1e-6);
    EXPECT_NEAR((0 + a*a) / 2, rsp.avg_succ_angle_sq_err, 1e-6);
    EXPECT_NEAR(2./5, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(1./5, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(1./5, rsp.none_rate, 1e-6);
}

TEST_F(ResponseFunctionTest, EmptyScenesOnly) {
    SetResult r;
    r.put("empty_scene_1", at_close);
    SetGroundTruth g;
    g["empty_scene_1"] = empty_scene;
    g["empty_scene_2"] = empty_scene;
    sse_response_function(r, g, rsp);
    EXPECT_TRUE(isnan(rsp.avg_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_err));
    EXPECT_TRUE(isnan(rsp.avg_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_angle_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_trans_sq_err));
    EXPECT_TRUE(isnan(rsp.avg_succ_trans_sq_err));
    EXPECT_NEAR(0, rsp.succ_rate, 1e-6);
    EXPECT_NEAR(0.5, rsp.mislabel_rate, 1e-6);
    EXPECT_NEAR(0.5, rsp.none_rate, 1e-6);
}
