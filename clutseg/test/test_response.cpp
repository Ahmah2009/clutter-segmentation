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
    Guess hm_perfect;
    Guess at_ge_max_angle;
    Guess at_ge_max_trans;
    Guess at_ge_max_trans_angle;
    Guess hm_ge_max_trans_angle;
    Guess jc_ge_max_trans_angle;
    Guess it_ge_max_angle;
    Guess it_ge_max_trans;
    Guess at_close;
    Guess it_close_fp;
    Guess hm_close;
    Guess jc_close;
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
            at_hm_jc.read(p / "ias_kinect_test_grounded_21/at_hm_jc/image_00008.png.ground.yaml");
            Pose atp = poseRtToPose(at_hm_jc.posesOf("assam_tea")[0]);
            Pose hmp = poseRtToPose(at_hm_jc.posesOf("haltbare_milch")[0]);
            Pose jcp = poseRtToPose(at_hm_jc.posesOf("jacobs_coffee")[0]);
            at_perfect = createGuess("assam_tea", atp);
            hm_perfect = createGuess("haltbare_milch", hmp);
            at_ge_max_angle = createGuess("assam_tea", rotatePose(atp, randomOrientation(M_PI / 8)));
            at_ge_max_trans = createGuess("assam_tea", translatePose(atp, (Mat_<double>(3, 1) << 0.02, 0.03, 0.01)));
            at_ge_max_trans_angle = createGuess("assam_tea", translatePose(rotatePose(atp, randomOrientation(M_PI / 8)), (Mat_<double>(3, 1) << 0.02, 0.02, 0.03)));
            hm_ge_max_trans_angle = createGuess("haltbare_milch", translatePose(rotatePose(hmp, randomOrientation(M_PI / 8)), (Mat_<double>(3, 1) << 0.02, 0.02, 0.03)));
            jc_ge_max_trans_angle = createGuess("jacobs_coffee", translatePose(rotatePose(jcp, randomOrientation(M_PI / 8)), (Mat_<double>(3, 1) << 0.02, 0.02, 0.03)));
            it_ge_max_angle = createGuess("icedtea", rotatePose(atp, randomOrientation(M_PI / 8)));
            it_ge_max_trans = createGuess("icedtea", translatePose(atp, (Mat_<double>(3, 1) << 0.02, 0.03, 0.01)));
            at_close = createGuess("assam_tea", translatePose(rotatePose(atp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
            it_close_fp = createGuess("icedtea", translatePose(rotatePose(atp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
            hm_close = createGuess("haltbare_milch", translatePose(rotatePose(hmp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
            jc_close = createGuess("jacobs_coffee", translatePose(rotatePose(jcp, randomOrientation(M_PI / 18)), (Mat_<double>(3, 1) << 0.02, 0.01, 0.01)));
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

        templateNames.insert("assam_tea");
        templateNames.insert("haltbare_milch");
        templateNames.insert("icedtea");
        templateNames.insert("jacobs_coffee");
    }

    CutSseResponseFunction sse_response_function;
    PoseRT pose;
    Ptr<TexturedObject> object;
    string img_name_single;
    SetGroundTruth ground_single;
    Response rsp;
    set<string> templateNames;

    void expect_sse_response_single(float expected, PoseRT est_pose) {
        Guess guess(object, poseRtToPose(est_pose), Mat(), Mat(), Mat());
        SetResult result_single;
        result_single[img_name_single] = Result(guess);
        sse_response_function(result_single, ground_single, templateNames, rsp);
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
    Guess guess_1(object, poseRtToPose(pose), Mat(), Mat(), Mat());
    Guess guess_2(object, poseRtToPose(pose_2), Mat(), Mat(), Mat());
    result[img_name_single] = Result(guess_1);
    result[img_name_single].detect_choices.push_back(guess_1);
    result[img_name_2] = Result(guess_2);
    result[img_name_2].detect_choices.push_back(guess_2);

    sse_response_function(result, ground_double, templateNames, rsp); 

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
    EXPECT_NEAR(1.0, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
}

TEST_F(ResponseFunctionTest, PerfectEstimatesOnly) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_perfect);
    r["at_hm_jc_1"].detect_choices.push_back(at_perfect);
    r["at_hm_jc_2"] = Result(at_perfect);
    r["at_hm_jc_2"].detect_choices.push_back(at_perfect);
    r["at_hm_jc_3"] = Result(at_perfect);
    r["at_hm_jc_3"].detect_choices.push_back(at_perfect);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);

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
    EXPECT_NEAR(1, rsp.locate_sipc.score(), 1e-6);
    EXPECT_NEAR(1./3, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
}

TEST_F(ResponseFunctionTest, BadEstimatesOnly) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_ge_max_angle);
    r["at_hm_jc_1"].detect_choices.push_back(at_ge_max_angle);
    r["at_hm_jc_2"] = Result(at_ge_max_trans);
    r["at_hm_jc_2"].detect_choices.push_back(at_ge_max_trans);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);

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
    EXPECT_NEAR(1./3, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
    // Correct labels produced, and half score for poses, so get 75% of points
    // in total. 
    EXPECT_NEAR(0.75, rsp.locate_sipc.score(), 1e-6);
}

TEST_F(ResponseFunctionTest, ReallyBadEstimatesOnly) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_ge_max_trans_angle);
    r["at_hm_jc_1"].detect_choices.push_back(at_ge_max_trans_angle);
    r["at_hm_jc_2"] = Result(at_ge_max_trans_angle);
    r["at_hm_jc_2"].detect_choices.push_back(at_ge_max_trans_angle);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);

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
    rsp.locate_sipc.print();
    EXPECT_NEAR(1./3, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    // Correct labels produced, but no score for poses, so get only half of the
    // points in total. 
    EXPECT_NEAR(0.5, rsp.locate_sipc.score(), 1e-6);
}


TEST_F(ResponseFunctionTest, NonesOnly) {
    SetResult r;
    r["at_hm_jc_1"] = Result();
    r["at_hm_jc_2"] = Result();
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);

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
    EXPECT_NEAR(0, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
    // Zero score when no choice was made
    EXPECT_NEAR(0, rsp.locate_sipc.score(), 1e-6);
}

/** Consider a scene where the recognizer does not make any choice at all.
 * We cannot update any error statistics that involve angle and translation error.
 * We may not include these scenes when averaging later, otherwise we could improve
 * on these error statistics by return more NONEs.
 */
TEST_F(ResponseFunctionTest, NonesDoNotPullDownAverage) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_max_trans_angle);
    r["at_hm_jc_1"].detect_choices.push_back(at_max_trans_angle);
    r["at_hm_jc_2"] = Result();
    r["at_hm_jc_3"] = Result();
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);
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
    EXPECT_NEAR(1./9, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(0, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
}

/** Handling of the case in which an object has been labeled that is not even
 * on the scene. */
TEST_F(ResponseFunctionTest, MislabelingsOnly) {
    SetResult r;
    r["at_hm_jc_1"] = Result(it_close_fp);
    r["at_hm_jc_1"].detect_choices.push_back(it_close_fp);
    r["at_hm_jc_2"] = Result(it_ge_max_angle);
    r["at_hm_jc_2"].detect_choices.push_back(it_ge_max_angle);
    r["at_hm_jc_3"] = Result(it_ge_max_trans);
    r["at_hm_jc_3"].detect_choices.push_back(it_ge_max_trans);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);
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
    EXPECT_NEAR(0, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(1, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
}

 /* Realistic example with all kinds of cases occuring. */
TEST_F(ResponseFunctionTest, PerfectNoneMislabelSuccessFail) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_perfect);
    r["at_hm_jc_1"].detect_choices.push_back(at_perfect);
    r["at_hm_jc_2"] = Result();
    r["at_hm_jc_3"] = Result(it_close_fp);
    r["at_hm_jc_3"].detect_choices.push_back(it_close_fp);
    r["at_hm_jc_4"] = Result(at_close);
    r["at_hm_jc_4"].detect_choices.push_back(at_close);
    r["at_hm_jc_5"] = Result(at_ge_max_trans_angle);
    r["at_hm_jc_5"].detect_choices.push_back(at_ge_max_trans_angle);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    g["at_hm_jc_4"] = at_hm_jc;
    g["at_hm_jc_5"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);
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
    EXPECT_NEAR(3./15, rsp.detect_tp_rate(), 1e-6);
    EXPECT_NEAR(1./5, rsp.detect_fp_rate(), 1e-6);
    rsp.locate_sipc.print();
}

TEST_F(ResponseFunctionTest, EmptyScenesOnly) {
    SetResult r;
    r["empty_scene_1"] = Result(at_close);
    r["empty_scene_2"] = Result();
    SetGroundTruth g;
    g["empty_scene_1"] = empty_scene;
    g["empty_scene_2"] = empty_scene;
    sse_response_function(r, g, templateNames, rsp);
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
    EXPECT_NEAR(0, rsp.none_rate, 1e-6);
    EXPECT_NEAR(0.5, rsp.locate_sipc.score(), 1e-6);
    rsp.locate_sipc.print();
}

TEST_F(ResponseFunctionTest, DetectSipcClassification) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_perfect);
    r["at_hm_jc_1"].detect_choices.push_back(at_ge_max_trans_angle);
    r["at_hm_jc_1"].detect_choices.push_back(hm_ge_max_trans_angle);
    r["at_hm_jc_1"].detect_choices.push_back(jc_ge_max_trans_angle);
    r["at_hm_jc_2"] = Result(hm_close);
    r["at_hm_jc_2"].detect_choices.push_back(hm_ge_max_trans_angle);
    r["at_hm_jc_2"].detect_choices.push_back(jc_ge_max_trans_angle);
    r["at_hm_jc_3"] = Result(it_close_fp);
    r["at_hm_jc_3"].detect_choices.push_back(it_close_fp);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);
    // max. class. score 3 per image. Image 1: 3/3, Image 2: 1.5/3, Image 3: 0/3
    // classification score weighted by 0.5
    EXPECT_NEAR(0.5 * 4.5 / 9, rsp.detect_sipc.score(), 1e-6);
}

TEST_F(ResponseFunctionTest, DetectSipcPose) {
    SetResult r;
    r["at_hm_jc_1"] = Result(at_perfect);
    r["at_hm_jc_1"].detect_choices.push_back(at_perfect);
    r["at_hm_jc_1"].detect_choices.push_back(hm_ge_max_trans_angle);
    r["at_hm_jc_1"].detect_choices.push_back(jc_ge_max_trans_angle);
    r["at_hm_jc_2"] = Result(it_close_fp); // must not matter!
    r["at_hm_jc_2"].detect_choices.push_back(hm_perfect);
    r["at_hm_jc_2"].detect_choices.push_back(jc_ge_max_trans_angle);
    r["at_hm_jc_3"] = Result(it_close_fp);
    r["at_hm_jc_3"].detect_choices.push_back(it_close_fp);
    SetGroundTruth g;
    g["at_hm_jc_1"] = at_hm_jc;
    g["at_hm_jc_2"] = at_hm_jc;
    g["at_hm_jc_3"] = at_hm_jc;
    sse_response_function(r, g, templateNames, rsp);
    // max. class. score 3 per image. Image 1: 3/3, Image 2: 1.5/3, Image 3: 0/3
    // max. pose score 3 per image. Image 1: 1/3, Image 2: 1/3, Image 3: 0/3
    // classification score 4.5/9 weighted by 0.5, 
    // pose score 2/9 weighted by 0.5, 
    EXPECT_NEAR(0.5 * (4.5 + 2) / 9, rsp.detect_sipc.score(), 1e-6);

}
