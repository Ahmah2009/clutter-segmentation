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

TEST(test_ground, label_read_many) {
    FileStorage fs("./data/image_00022.detect_choices.yaml.gz", FileStorage::READ);
    vector<Label> nps;
    for (FileNodeIterator n_it = fs.root().begin(); n_it != fs.root().end(); n_it++) {
        Label np((*n_it).name());
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

TEST(test_ground, read_ground_truth_without_poses) {
    GroundTruth  m = loadGroundTruthWithoutPoses("./data/ground-truth.txt");
    LabelSet s = m["t0000.png"];

    BOOST_FOREACH(const Label & np, s.labels) {
        EXPECT_FALSE(np.pose.estimated);
    }
    EXPECT_TRUE(s.onScene("teas_tea"));
    EXPECT_TRUE(s.onScene("fat_free_milk"));
    EXPECT_FALSE(s.onScene("downy"));
    EXPECT_FALSE(s.onScene(""));
    EXPECT_FALSE(s.onScene(" "));
}

