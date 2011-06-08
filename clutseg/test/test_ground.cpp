/*
 * Author: Julius Adorf
 */

#include "clutseg/ground.h"

#include <boost/foreach.hpp>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace clutseg;

TEST(GroundTest, LabeledPoseRead) {
    LabeledPose np;
    FileStorage fs("./data/image_00042.png.pose.yaml", FileStorage::READ);
    np.read(fs.getFirstTopLevelNode());
    fs.release();
    EXPECT_EQ("pose", np.name);
    EXPECT_FLOAT_EQ(2.3623705429591477, np.pose.rvec.at<double>(0, 0));
}

TEST(GroundTest, LabeledPoseReadMany) {
    FileStorage fs("./data/image_00022.detect_choices.yaml.gz", FileStorage::READ);
    vector<LabeledPose> nps;
    for (FileNodeIterator n_it = fs.root().begin(); n_it != fs.root().end(); n_it++) {
        LabeledPose np((*n_it).name());
        np.pose.read(*n_it);
        np.pose.estimated = true;
        nps.push_back(np);
    }
    fs.release();
    EXPECT_EQ("haltbare_milch", nps[0].name);
    // This is really annoying, depending on whether one writes Pose or PoseRT
    // it gets either double or float type. If accessed incorrectly, the numbers
    // are trash.
    EXPECT_FLOAT_EQ(4.48224425e-01, nps[0].pose.rvec.at<float>(0, 0));
    EXPECT_EQ("assam_tea", nps[1].name);
    EXPECT_FLOAT_EQ(4.51702595e-01, nps[1].pose.rvec.at<float>(0, 0));
}

TEST(GroundTest, ReadSetGroundTruthWithoutPoses) {
    SetGroundTruth m = loadSetGroundTruthWithoutPoses("./data/testdesc.txt");
    GroundTruth s = m["t0000.png"];

    BOOST_FOREACH(const LabeledPose & np, s.labels) {
        EXPECT_FALSE(np.pose.estimated);
    }
    EXPECT_TRUE(s.onScene("teas_tea"));
    EXPECT_TRUE(s.onScene("fat_free_milk"));
    EXPECT_FALSE(s.onScene("downy"));
    EXPECT_FALSE(s.onScene(""));
    EXPECT_FALSE(s.onScene(" "));
}

