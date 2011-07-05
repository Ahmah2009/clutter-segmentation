/**
 * Author: Julius Adorf
 */
#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <cv.h>
    #include <iostream>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl_ros/transforms.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/filters/radius_outlier_removal.h>
    #include <tod/core/TexturedObject.h>
    #include <tod/core/TrainingBase.h>
    #include <tod/detecting/Loader.h>
    #include <tod/detecting/Tools.h>
    // Put this include at the very end, otherwise the compiler will
    // complain with lengthy messsages
    #include <opencv2/core/eigen.hpp>
    #include <opencv2/highgui/highgui.hpp>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace cv;
using namespace pcl;
using namespace std;
using namespace tod;

void pcd_xy_histogram(const PointCloud<PointXYZ> & model, Mat & xy_hist_bgr, float x_min, float x_max, float x_w, float y_min, float y_max, float y_w) {
    int r = ceil((y_max - y_min) / y_w);
    int c = ceil((x_max - x_min) / x_w); 
    Mat xy_hist = Mat::zeros(r, c, CV_32FC1);
    float m = 0.0;
    BOOST_FOREACH(const PointXYZ p, model) {
        int i = (p.y - y_min) / y_w;
        int j = (p.x - x_min) / x_w; 
        if (i >= 0 && i < c && j >= 0 && j < r) {
            xy_hist.at<float>(i, j) += 1.0f;
            cout << boost::format("(%6.3f, %6.3f) -> H(%d, %d) = %6.2f") % p.x % p.y % i % j % xy_hist.at<float>(i, j) << endl;
            m = m >= xy_hist.at<float>(i, j) ? m : xy_hist.at<float>(i, j);
        }
    }
    cout << "x_min = " << x_min << endl;
    cout << "x_max = " << x_max << endl;
    cout << "x_w   = " << x_w << endl;
    cout << "y_min = " << y_min << endl;
    cout << "y_max = " << y_min << endl;
    cout << "y_w   = " << y_w << endl;
    cout << "r = " << r << endl;
    cout << "c = " << c << endl;
    cout << "m = " << m << endl;
    m = 12;
    // Normalize
    xy_hist *= 255 / m;
    // Invert intensities 
    xy_hist = 255 - xy_hist; 
    // Convert to color
    cvtColor(xy_hist, xy_hist_bgr, CV_GRAY2BGR);
    // Draw x-axis (i.e. y = 0) in red
    line(xy_hist_bgr, Point(max(0, int(-y_min / y_w)), 0), Point(max(int(-y_min / y_w), 0), c), Scalar(0, 0, 255));
    // Draw y-axis (i.e. x = 0) in green
    line(xy_hist_bgr, Point(0, min(int(-x_min / x_w), r)), Point(r, min(int(-x_min / x_w), c)), Scalar(0, 255, 0));
}

int main(int argc, char **argv) {
    if (argc != 4 && argc != 11) {
        cerr << "Usage: assemble_model <base> <object> <dst> [<xmin> <xmax> <xw> <ymin> <ymax> <yw> <hist_xy>]" << endl;
        cerr << endl <<
            "Reads in a model from the modelbase (a.k.a. TrainingBase) and generates a\n"
            "point cloud file that contains all model points (corresponding to features)\n"
            "described in its object coordinate system." << endl;
        return -1;
    }

    // Loading all of them is a little bit of overkill
    vector<Ptr<TexturedObject> > objects;
    Loader loader(argv[1]);
    loader.readTexturedObjects(objects);

    int i = -1;
    Ptr<TexturedObject> object = NULL;
    BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects) {
        i++;
        if (obj->name == argv[2]) {
            object = obj;
        }
    }

    if (i == -1) {
        cerr << "Object " << argv[2] << " does not exist." << endl;
        cerr << "Objects in modelbase: ";
        BOOST_FOREACH(const Ptr<TexturedObject> & obj, objects) {
            cerr << obj->name << " ";
        }
        cerr << endl;
        return -1;
    }

    PointCloud<PointXYZ> model;
    BOOST_FOREACH(const Features3d & f3d, object->observations) {
        // Annoying, the model features are stored in view coordinates need to
        // retrieve the object-view transformation for each view.  The
        // object-view transformation is stored as the extrinsic parameters of
        // the camera. We need to invert it to get hold of the sought-after
        // view-object transformation that brings points in view coordinates
        // into the object coordinate system. <R, t> pose is the object-view
        // transformation.
        PoseRT pose = f3d.camera().pose;

        vector<Point3d> op;
        BOOST_FOREACH(const Point3f pf, f3d.cloud()) {
            Point3d pd;
            pd.x = (double) pf.x;
            pd.y = (double) pf.y;
            pd.z = (double) pf.z;
            op.push_back(pd);
        }
        Mat mop(op);
        cv::Mat R;
        PoseRT inverted = Tools::invert(pose);
        cv::Rodrigues(inverted.rvec, R);
        cv::transform(mop, mop, R);
        BOOST_FOREACH(Point3d & pd, op) {
            pd += inverted.tvec.at<cv::Point3d>();
        }

        PointCloud<PointXYZ> view_cloud;
        cout << f3d.cloud().size() << endl;
        cout << f3d.features().keypoints.size() << endl;
        BOOST_FOREACH(const Point3d pd, op) {
            PointXYZ p;
            p.x = pd.x;
            p.y = pd.y;
            p.z = pd.z;
            view_cloud.push_back(p);
        }

        model += view_cloud;

        /* This is a second way to do the transform, leads to the same results
        vector<int> indices;
        pcl::removeNaNFromPointCloud(view_cloud, view_cloud, indices);

        Eigen::Matrix<float, 3, 3> R;
        Eigen::Vector3f t;
        cv::cv2eigen(pose.tvec, t);
        cv::Mat cvR;
        cv::Rodrigues(pose.rvec, cvR);
        cv::cv2eigen(cvR, R);

        Eigen::Affine3f translate(Eigen::Translation3f(-t));
        pcl::transformPointCloud(view_cloud, view_cloud, translate);

        Eigen::Affine3f rotate(Eigen::AngleAxisf(R.transpose()));
        pcl::transformPointCloud(view_cloud, view_cloud, rotate);

        model += view_cloud; */
    }

    io::savePCDFileASCII(argv[3], model);

    if (argc > 8) {
        Mat xy_hist_bgr;
        pcd_xy_histogram(model, xy_hist_bgr,
                atof(argv[4]), atof(argv[5]), atof(argv[6]),
                atof(argv[7]), atof(argv[8]), atof(argv[9]));
        imwrite(argv[10], xy_hist_bgr);
    }
    
    return 0;
}
