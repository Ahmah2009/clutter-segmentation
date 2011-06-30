/**
 * Author: Julius Adorf
 */

#include "clutseg/experiment.h"
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <cv.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <tod/core/TrainingBase.h>
#include <tod/detecting/Loader.h>
#include <tod/detecting/Matcher.h>
#include <tod/training/feature_extraction.h>

using namespace clutseg;
using namespace cv;
using namespace std;
using namespace tod;

namespace bfs = boost::filesystem;

int main(int argc, char **argv) {
    if (argc != 6) { 
        cerr << "Usage: draw_correspondences <modelbase> <trainbase> <object> <query-image> <output-image>" << endl;
        return -1;
    }

    bfs::path base_dir = argv[1];
    bfs::path trainbase_dir = argv[2];
    string name = argv[3];
    Mat query_img = imread(argv[4], CV_LOAD_IMAGE_GRAYSCALE);
    string out_p = argv[5];
    assert(!query_img.empty());

    MatcherParameters mp;
    mp.type = "LSH-BINARY";
    mp.doRatioTest = 0;
    mp.ratioThreshold = 0.8;
    mp.knn = 3;

    vector<Ptr<TexturedObject> > objects;
    Loader loader(base_dir.string());
    loader.readTexturedObjects(objects);
    TrainingBase base(objects);

    FeatureExtractionParams fep;
    readFeParams(base_dir / "features.config.yaml", fep); 
    Ptr<FeatureExtractor> fe = FeatureExtractor::create(fep);
    Features2d f2d;
    f2d.image = query_img;
    fe->detectAndExtract(f2d);

    Ptr<Matcher> matcher = Matcher::create(mp);
    matcher->add(base);
    matcher->match(f2d.descriptors);


    for (size_t objectInd = 0; objectInd < base.size(); objectInd++) {
        const Ptr<TexturedObject> & obj = base.getObject(objectInd);
        if (obj->name == name) {
            tod::Matches matches;
            size_t img_idx = 0;
            for (size_t i = 0; i < obj->observations.size(); i++) {
                tod::Matches imageMatches;
                matcher->getImageMatches(objectInd, i, imageMatches);
                if (imageMatches.size() >= matches.size()) {
                    cout << imageMatches.size() << endl;
                    matches = imageMatches;
                    img_idx = i;
                }
            }
        
            // Find image img_idx
            bfs::path nm_img = bfs::path(trainbase_dir) / name / obj->observations[img_idx].features().image_name;
            Mat model_img;
            model_img = imread(nm_img.string());
            assert(!model_img.empty());

            Mat c;
            cv::drawMatches(query_img, f2d.keypoints, model_img,
                        obj->observations[img_idx].features().keypoints, matches,
                        c, Scalar(255, 0, 0), Scalar(0, 0, 255));

            /* float scale = 0.75;
            Mat sc = Mat::zeros(c.rows * scale, c.cols * scale, CV_8UC3);
            resize(c, sc, sc.size(), 0, 0, CV_INTER_LANCZOS4);*/

            imwrite(out_p, c);


            break;
        }
    }

    return 0;
}
