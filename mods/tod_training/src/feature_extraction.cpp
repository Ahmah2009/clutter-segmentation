/*
 * feature_extraction.cpp
 *
 *  Created on: Dec 7, 2010
 *      Author: erublee
 */
#include <boost/foreach.hpp>
#include <tod/training/feature_extraction.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <rbrief/detectors.h>
#include <typeinfo>

using std::cout;
using namespace cv;
namespace tod
{

    void FeatureExtractionParams::write(cv::FileStorage & fs) const
    {
        cvWriteComment(*fs, "FeatureExtractionParams", 0);
        fs << "{";
        fs << "detector_type" << detector_type << "extractor_type" <<
            extractor_type << "descriptor_type" << descriptor_type;
        fs << "detector_params" << "{";
        std::map < std::string, double >::const_iterator it =
            detector_params.begin();
        for (; it != detector_params.end(); ++it)
        {
            fs << it->first << it->second;
        }
        fs << "}";
        fs << "extractor_params" << "{";
        it = extractor_params.begin();
        for (; it != extractor_params.end(); ++it)
        {
            fs << it->first << it->second;
        }
        fs << "}";
        fs << "}";

    }
    void FeatureExtractionParams::read(const cv::FileNode & fn)
    {
        detector_type = (string) fn["detector_type"];
        extractor_type = (string) fn["extractor_type"];
        descriptor_type = (string) fn["descriptor_type"];

        cv::FileNode params = fn["detector_params"];
        CV_Assert(params.type() == cv::FileNode::MAP);
        cv::FileNodeIterator it = params.begin();
        for (; it != params.end(); ++it) {
            detector_params[(*it).name()] = (double) (*it);
            //cout << "read:" << (*it).name() << " = " << detector_params[(*it).name()] << endl;
        }
        params = fn["extractor_params"];
        CV_Assert(params.type() == cv::FileNode::MAP);

        it = params.begin();
        for (; it != params.end(); ++it) {
            extractor_params[(*it).name()] = (double) (*it);
            //cout << "read:" << (*it).name() << " = " << extractor_params[(*it).name()] << endl;
        }

    }

    FeatureExtractionParams FeatureExtractionParams::CreateSampleParams()
    {
        FeatureExtractionParams params;
        params.descriptor_type = "rBRIEF";
        params.detector_type = "FAST";
        params.extractor_type = "multi-scale";
        params.extractor_params["octaves"] = 3;
        params.detector_params["min_features"] = 500;
        params.detector_params["max_features"] = 700;
        params.detector_params["threshold"] = 200;
        return params;
    }

    const std::string FeatureExtractionParams::YAML_NODE_NAME =
        "feature_extraction_params";

    FeatureDetector *FeatureExtractor::
        createDetector(const string & detectorType, float threshold, detector_stats & stats)
    {
        FeatureDetector *fd = 0;
        if (!detectorType.compare("FAST")) {
            fd = new FastFeatureDetector(threshold /*threshold */ ,
                                         true /*nonmax_suppression */ );
            stats.internal_detector = "cv::FastFeatureDetector";
            stats.pm_threshold_used = true;
        } else if (!detectorType.compare("STAR")) {
            fd = new StarFeatureDetector(16 /*max_size */ ,
                                         (int) threshold
                                         /*response_threshold */ ,
                                         10 /*line_threshold_projected */ ,
                                         8 /*line_threshold_binarized */ ,
                                         5 /*suppress_nonmax_size */ );
            stats.internal_detector = "cv::StarFeatureDetector";
            stats.pm_threshold_used = true;
        } else if (!detectorType.compare("SIFT")) {
            fd = new SiftFeatureDetector(SIFT::DetectorParams::
                                         GET_DEFAULT_THRESHOLD(),
                                         SIFT::DetectorParams::
                                         GET_DEFAULT_EDGE_THRESHOLD());
            stats.internal_detector = "cv::SiftFeatureDetector";
        } else if (!detectorType.compare("SURF")) {
            fd = new SurfFeatureDetector(threshold /*hessian_threshold */ ,
                                         5 /*octaves */ ,
                                         4 /*octave_layers */ );
            stats.internal_detector = "cv::SurfFeatureDetector";
            stats.pm_threshold_used = true;
        } else if (!detectorType.compare("MSER")) {
            fd = new MserFeatureDetector(5 /*delta */ , 60 /*min_area */ ,
                                         14400 /*_max_area*/ ,
                                         0.25f /*max_variation */ ,
                                         0.2 /*min_diversity */ ,
                                         200 /*max_evolution */ ,
                                         threshold /*area_threshold */ ,
                                         0.003 /*min_margin */ ,
                                         5 /*edge_blur_size */ );
            stats.internal_detector = "cv::MserFeatureDetector";
            stats.pm_threshold_used = true;
        } else if (!detectorType.compare("GFTT")) {
            fd = new GoodFeaturesToTrackDetector(1000 /*maxCorners */ ,
                                                 threshold /*qualityLevel */ ,
                                                 1. /*minDistance */ ,
                                                 3 /*int _blockSize */ ,
                                                 true /*useHarrisDetector */ ,
                                                 0.04 /*k */ );
            stats.internal_detector = "cv::GoodFeaturesToTrackDetector";
            stats.pm_threshold_used = true;
        } else {
            assert(0);
        }
        return fd;
    }

    FeatureExtractor *FeatureExtractor::create(FeatureExtractionParams params) {
        detector_stats stats;
        return create(params, stats);
    }

    FeatureExtractor *FeatureExtractor::create(FeatureExtractionParams params,
                                               detector_stats & stats)
    {
        FeatureExtractor *fe = 0;
        int min_features = params.detector_params["min_features"];
        int max_features = params.detector_params["max_features"];
        cv::Ptr < FeatureDetector > detector;

        // ORB is treated differently
        if ((params.extractor_type == "ORB")
            || (params.detector_type == "ORB")
            || (params.descriptor_type == "ORB")) {
            params.extractor_type = "ORB";
            //  params.detector_type = "ORB";
            params.descriptor_type = "ORB";
        }

        if (params.detector_type == "DynamicFAST") {
            detector =
                new DynamicAdaptedFeatureDetector(new rbrief::
                                                  RBriefFastAdjuster(80, true,
                                                                     10, 255),
                                                  min_features, max_features,
                                                  200);
            stats.internal_detector = "cv::DynamicAdaptedFeatureDetector with tod::RBriefFastAdjuster";
            stats.pm_min_features_used = true;
            stats.pm_max_features_used = true;
            stats.pm_threshold_used = false;
            stats.pm_octaves_used = false;
        } else if (params.detector_type == "DynamicSTAR") {
            detector =
                new DynamicAdaptedFeatureDetector(new StarAdjuster(),
                                                  min_features, max_features,
                                                  200);
            stats.internal_detector = "cv::DynamicAdaptedFeatureDetector with cv::StarAdjuster";
            stats.pm_min_features_used = true;
            stats.pm_max_features_used = true;
            stats.pm_threshold_used = false;
            stats.pm_octaves_used = false;
        } else if (params.detector_type == "DynamicSURF") {
            detector =
                new DynamicAdaptedFeatureDetector(new SurfAdjuster(),
                                                  min_features, max_features,
                                                  200);
            stats.internal_detector = "cv::DynamicAdaptedFeatureDetector with cv::SurfAdjuster";
            stats.pm_min_features_used = true;
            stats.pm_max_features_used = true;
            stats.pm_threshold_used = false;
            stats.pm_octaves_used = false;
        } else if (std::string::npos != params.detector_type.find("ORB")) {
            // do nothing here?
            stats.internal_detector = "<none>";
        } else if (params.detector_params.end() !=
                   params.detector_params.find("threshold")) {
            detector =
                FeatureExtractor::createDetector(params.detector_type,
                                                 params.
                                                 detector_params
                                                 ["threshold"], stats);
            stats.internal_detector = typeid(*detector).name();
            stats.pm_min_features_used = false;
            stats.pm_max_features_used = false;
            stats.pm_threshold_used = true;
            stats.pm_octaves_used = false;
        } else {
            detector = FeatureDetector::create(params.detector_type);
            stats.internal_detector = typeid(*detector).name();
            stats.pm_min_features_used = false;
            stats.pm_max_features_used = false;
            stats.pm_threshold_used = false;
            stats.pm_octaves_used = false;
        }

        // Define the extractor
        cv::Ptr < DescriptorExtractor > extractor;
        if (params.descriptor_type == "rBRIEF") {
            rbrief::RBriefDescriptorExtractor * rbrief_extractor =
                new rbrief::RBriefDescriptorExtractor();
            // Set the patch size
            //switch ((int) params.extractor_params["patch_size"])
            switch (32) {
                case 16:
                    rbrief_extractor->
                        setPatchSize(rbrief::RBriefDescriptorExtractor::
                                     PATCH_16);
                    break;
                case 32:
                    rbrief_extractor->
                        setPatchSize(rbrief::RBriefDescriptorExtractor::
                                     PATCH_32);
                    break;
                case 48:
                    rbrief_extractor->
                        setPatchSize(rbrief::RBriefDescriptorExtractor::
                                     PATCH_48);
                    break;
            }
            rbrief_extractor->
                setPatchSize(rbrief::RBriefDescriptorExtractor::
                             PATCH_LEARNED);
            // Set the steerer
            //int kernel_size = params.extractor_params["half_kernel_size"];
            int half_kernel_size = 15;
            Ptr < rbrief::KeypointSteerer >
                steerer(new rbrief::IntensityCentroid(half_kernel_size));
            rbrief_extractor->setSteerer(steerer);
            extractor = rbrief_extractor;
            stats.internal_extractor = "rbrief::RBriefDescriptorExtractor";
        } else if (params.descriptor_type == "ORB") {
            // do nothing here?
            stats.internal_extractor = "<none>";
        } else {
            extractor = DescriptorExtractor::create(params.descriptor_type);
            stats.internal_extractor = typeid(extractor).name();
        }

        if ((params.extractor_type != "ORB") && extractor.empty()) {
            stats.extractor = "<bad>";
            throw std::runtime_error("bad extractor");
        }
        if (params.extractor_type == "multi-scale") {
            fe = new MultiscaleExtractor(detector, extractor,
                                         params.extractor_params["octaves"]);
            stats.pm_octaves_used = true;
            stats.extractor = "tod::MultiscaleExtractor";
        } else if (params.extractor_type == "sequential") {
            fe = new SequentialExtractor(detector, extractor);
            stats.extractor = "tod::SequentialExtractor";
        } else if (params.extractor_type == "ORB") {
            if (std::string::npos != params.detector_type.find("gridded")) {
                fe = new OrbExtractor(1.2, params.extractor_params["octaves"],
                                      max_features,
                                      rbrief::RBriefDescriptorExtractor::
                                      PATCH_LEARNED, true,
                                      OrbExtractor::ANMSFast);
                stats.extractor = "tod::OrbExtractor with AMMSFast";
            } else {
                fe = new OrbExtractor(1.2, params.extractor_params["octaves"],
                                      max_features,
                                      rbrief::RBriefDescriptorExtractor::
                                      PATCH_LEARNED, true,
                                      OrbExtractor::HarrisFast);
                stats.extractor = "tod::OrbExtractor with HarrisFast";
            }
            stats.internal_extractor = "<none>";
            stats.pm_threshold_used = false;
            stats.pm_min_features_used = false;
            stats.pm_max_features_used = true;
            stats.pm_octaves_used = true;
        }
        return fe;

    }

  MultiscaleExtractor::MultiscaleExtractor(const cv::Ptr < cv::FeatureDetector > &d, const cv::Ptr < cv::DescriptorExtractor > &e, int n_octaves):
    detector_(d), extractor_(e), n_octaves_(n_octaves)
    {

    }

    void MultiscaleExtractor::detectAndExtract(Features2d & features) const
    {
        int octaves = n_octaves_;
        Mat image = features.image.clone();

        float scale_factor = sqrt(2);
        Mat mask = features.mask.empty()? Mat() : features.mask.clone();

        float scale_x = 1.0f;
        float scale_y = 1.0f;
        for (int i = 0; i < octaves; i++)
        {
            vector < KeyPoint > kpts;
            Mat descriptors;
            detector_->detect(image, kpts, mask);
            extractor_->compute(image, kpts, descriptors);

            for (size_t j = 0; j < kpts.size(); j++)
            {
                kpts[j].pt.x *= scale_x;
                kpts[j].pt.y *= scale_y;
            }

            if (i < octaves - 1)
            {
                scale_x = features.image.cols / (image.cols / scale_factor);
                scale_y = features.image.rows / (image.rows / scale_factor);

                Size n_size(image.cols / scale_factor,
                            image.rows / scale_factor);
                resize(features.image, image, n_size);
                if (!features.mask.empty())
                    resize(features.mask, mask, n_size);
            }
            features.keypoints.insert(features.keypoints.end(), kpts.begin(),
                                      kpts.end());
            Mat n_desc(features.descriptors.rows + descriptors.rows,
                       extractor_->descriptorSize(),
                       extractor_->descriptorType());
            Mat top_desc(n_desc.
                         rowRange(Range(0, features.descriptors.rows)));
            Mat bottom_desc(n_desc.
                            rowRange(Range
                                     (features.descriptors.rows,
                                      features.descriptors.rows +
                                      descriptors.rows)));

            features.descriptors.copyTo(top_desc);
            descriptors.copyTo(bottom_desc);
            features.descriptors = n_desc;
#if 0
            imshow("octave", image);
            imshow("scaled mask", mask);
            waitKey(0);
#endif
        }
    }

    OrbExtractor::OrbExtractor(float scale_factor, int n_levels,
                               int n_desired_features,
                               rbrief::RBriefDescriptorExtractor::
                               PatchSize ps, bool steering,
                               DetectorType
                               detector):scale_factor_(scale_factor),
        n_levels_(n_levels > 0 ? n_levels : 1),
        n_desired_features_(n_desired_features), working_mats_(n_levels_),
        working_masks_(n_levels_), working_kpts_(n_levels_),
        working_descriptors_(n_levels_), rbriefs_(n_levels_),
        get_patches_(false)
    {
        for (size_t i = 0; i < rbriefs_.size(); i++) {
            rbriefs_[i] = new rbrief::RBriefDescriptorExtractor();
            rbriefs_[i]->setPatchSize(ps);
            if (steering)
                rbriefs_[i]->setSteerer(new rbrief::IntensityCentroid(15));
            else
                rbriefs_[i]->setSteerer(new rbrief::UniformSteerer(0));
            switch (detector) {
                case ANMSFast:
                    feature_detectors_.
                        push_back(new rbrief::
                                  ANMSFastDetector(n_desired_features_ *
                                                   (1 /
                                                    std::pow(scale_factor_,
                                                             i))));
                    break;
                case HarrisFast:
                    feature_detectors_.
                        push_back(new rbrief::
                                  SimpleFASTHarris(n_desired_features_ *
                                                   (1 /
                                                    std::pow(scale_factor_,
                                                             i)), true));
                    break;
                case Fast:
                    feature_detectors_.
                        push_back(new cv::FastFeatureDetector(20));
                    break;
            }
        }
    }

    void OrbExtractor::getPatches(std::vector < cv::Mat > &patches) const
    {
        patches = patches_;
    }
    void OrbExtractor::getPatches(cv::Mat & patches_out) const
    {
        if (patches_.empty())
            return;
        if (patches_out.empty())
            patches_out =
                cv::Mat(1, patches_.front().cols * 10,
                        patches_.front().type());
        cv::Mat temp_patch;
        BOOST_FOREACH(cv::Mat patch, patches_)
        {
            if (temp_patch.empty())
                patch.copyTo(temp_patch);
            else
                temp_patch.push_back(patch);
            if (temp_patch.rows == patches_out.cols) {
                cv::Mat tt = temp_patch.t();
                if (patches_out.rows == 1)
                    tt.copyTo(patches_out);
                else
                    patches_out.push_back(tt);
                temp_patch = Mat();
            }
        }
    }

    cv::Mat GetAffineRotationMat(float a, cv::Point t)
    {
        a = a * CV_PI / 180;
        cv::Mat ar =
            (cv::Mat_ < float >(3, 3) << cos(a), -sin(a), t.x, sin(a), cos(a),
             t.y, 0, 0, 1);
        return ar;
    }

    void GetPatches(cv::Size patch_size, const cv::Mat & image,
                    const std::vector < cv::KeyPoint > &kpts,
                    std::vector < cv::Mat > &patches)
    {
        patches.clear();
        cv::Rect roi(0, 0, patch_size.width + 10, patch_size.height + 10);
        cv::Rect image_roi(0, 0, image.size().width, image.size().height);
        BOOST_FOREACH(const cv::KeyPoint & kpt, kpts)
        {
            roi.x = kpt.pt.x - patch_size.width / 2 - 5;
            roi.y = kpt.pt.y - patch_size.height / 2 - 5;
            if ((image_roi & roi).area() != roi.area())
                continue;
            cv::Point t(roi.width / 2, roi.height / 2);
            cv::Point t2(patch_size.width / 2, patch_size.height / 2);
            cv::Mat w =
                GetAffineRotationMat(0, t2) * GetAffineRotationMat(-kpt.angle,
                                                                   Point())
                * GetAffineRotationMat(0, -t);
            cv::Mat patch;
            cv::warpPerspective(image(roi), patch, w, patch_size);
            patches.push_back(patch);
        }
    }

    void OrbExtractor::detectAndExtract(Features2d & features) const
    {
        detectAndExtract(features.image, features.keypoints,
                         features.descriptors, features.mask);
    }

    void OrbExtractor::detectAndExtract(const cv::Mat & image,
                                        std::vector < cv::KeyPoint >
                                        &keypoints, cv::Mat & descriptors,
                                        const cv::Mat & mask) const
    {
        watch_ = rbrief::StopWatch();
        std::vector < float >scales(n_levels_, 1);
        keypoints.clear();
        int zoom_level = 1;
        working_mats_[zoom_level] = image;
        working_masks_[zoom_level] = mask;
        //  omp_set_num_threads(1);
        //#pragma omp parallel for
        for (int i = 0; i < n_levels_; i++)
        {
            if (i != zoom_level) {
                watch_.startLap("scaling");
                float scale = 1 / std::pow(scale_factor_, i - zoom_level);
                scales[i] = scale;
                cv::resize(working_mats_[zoom_level], working_mats_[i],
                           cv::Size(), scale, scale, cv::INTER_AREA);
                if (!mask.empty())
                    cv::resize(working_masks_[zoom_level], working_masks_[i],
                               cv::Size(), scale, scale, cv::INTER_AREA);
                watch_.stopLap("scaling");
            }

            std::vector < cv::KeyPoint > &kpts = working_kpts_[i];
            cv::Mat & desc = working_descriptors_[i];
            watch_.startLap("detecting");
            feature_detectors_[i]->detect(working_mats_[i], kpts,
                                          working_masks_[i]);
            watch_.stopLap("detecting");
            watch_.startLap("compute");
            // +1 as it is the step size of the integral image
            rbriefs_[i]->setStepSize(working_mats_[i].cols + 1);
            rbriefs_[i]->compute(working_mats_[i], kpts, desc);
            watch_.stopLap("compute");
            if (i != zoom_level) {
                float scale = 1 / scales[i];
                for (size_t j = 0; j < kpts.size(); j++) {
                    kpts[j].pt *= scale;
                    kpts[j].octave = i;
                }

            }
        }
        for (int i = 0; i < n_levels_; i++) {
            keypoints.insert(keypoints.end(), working_kpts_[i].begin(),
                             working_kpts_[i].end());
            if (i == 0) {
                working_descriptors_[i].copyTo(descriptors);
            } else
                descriptors.push_back(working_descriptors_[i]);
        }

    }
    void OrbExtractor::debugDisplay() const
    {
        for (int i = 0; i < n_levels_; i++)
        {
            std::stringstream ss;
            ss << "scale " << 1 / std::pow(scale_factor_, i);
            imshow(ss.str(), working_mats_[i]);
        }
    }

    SequentialExtractor::SequentialExtractor(const cv::Ptr <
                                             cv::FeatureDetector > &d,
                                             const cv::Ptr <
                                             cv::DescriptorExtractor >
                                             &e):detector_(d), extractor_(e)
    {

    }
    void SequentialExtractor::detectAndExtract(Features2d & features) const
    {
        detector_->detect(features.image, features.keypoints, features.mask);
        extractor_->compute(features.image, features.keypoints,
                            features.descriptors);
    }

    FileExtractor::FileExtractor(const std::
                                 string & f2dname):f2dname_(f2dname)
    {

    }
    void FileExtractor::detectAndExtract(Features2d & features) const
    {
        cv::FileStorage fs(f2dname_, cv::FileStorage::READ);
        cv::read(fs["keypoints"], features.keypoints);
        //    fs["keypoints"] >> features.keypoints;
        fs["descriptors"] >> features.descriptors;
    }
    void KeyPointsToPoints(const KeypointVector & keypts,
                           std::vector < cv::Point2f > &pts)
    {
        pts.clear();
        pts.reserve(keypts.size());
        for (size_t i = 0; i < keypts.size(); i++) {
            pts.push_back(keypts[i].pt);
        }
    }

    void PointsToKeyPoints(const std::vector < cv::Point2f > &pts,
                           KeypointVector & kpts)
    {
        kpts.clear();
        kpts.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); i++) {
            kpts.push_back(KeyPoint(pts[i], 6.0));
        }
    }

}
