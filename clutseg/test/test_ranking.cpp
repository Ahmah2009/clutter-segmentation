/**
 * Author: Julius Adorf
 */

#include "test.h"

#include "clutseg/ranking.h"

#include <cv.h>
#include <gtest/gtest.h>
#include <vector>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

struct RankingTest : public ::testing::Test {

    void SetUp() {
        few_inliers_guess.inliers = vector<int>(9);
        many_inliers_guess.inliers = vector<int>(150);
        guesses.push_back(few_inliers_guess);
        guesses.push_back(many_inliers_guess);
    }

    UniformRanking uniform_ranking;
    MaxInliersRanking max_inliers_ranking;
    
    Guess few_inliers_guess;
    Guess many_inliers_guess;
    vector<Guess> guesses;

};

TEST_F(RankingTest, UniformRanking) {
    EXPECT_EQ(uniform_ranking(many_inliers_guess), uniform_ranking(few_inliers_guess));
}

// TODO: rename MaxInliersRanking, it's more or less obvious that more inliers are better

TEST_F(RankingTest, MaxInliersRanking) {
    EXPECT_GT(max_inliers_ranking(many_inliers_guess), max_inliers_ranking(few_inliers_guess));
}

TEST_F(RankingTest, UniformSort) {
    // I expect this to be a no-op
    Ptr<GuessRanking> r = new UniformRanking();
    GuessComparator cmp(r);
    EXPECT_LT(guesses[0].inliers.size(), guesses[1].inliers.size());
    sort(guesses.begin(), guesses.end(), cmp);
    EXPECT_LT(guesses[0].inliers.size(), guesses[1].inliers.size());
}

TEST_F(RankingTest, MaxInliersSort) {
    Ptr<GuessRanking> r = new MaxInliersRanking();
    GuessComparator cmp(r);
    EXPECT_LT(guesses[0].inliers.size(), guesses[1].inliers.size());
    sort(guesses.begin(), guesses.end(), cmp);
    EXPECT_GT(guesses[0].inliers.size(), guesses[1].inliers.size());
}

/*TEST_F(RankingTest, MaxInliersMax) {
    Ptr<GuessRanking> r = new MaxInliersRanking();
    GuessComparator cmp(r);
    max(guesses.begin(), guesses.end(), cmp);
}*/
