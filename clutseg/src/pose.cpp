/*
 * Author: Julius Adorf
 */

#include "clutseg/pose.h"

#include "clutseg/check.h"

#include "clutseg/gcc_diagnostic_disable.h"
    #include <boost/random/normal_distribution.hpp>
    #include <boost/random.hpp>
    #include <boost/foreach.hpp>
    #include <boost/format.hpp>
    #include <fiducial/fiducial.h>
    #include <opencv/cxeigen.hpp>
    #include <opencv2/calib3d/calib3d.hpp>
    #include <set>
    #include <utility>
#include "clutseg/gcc_diagnostic_enable.h"

using namespace boost;
using namespace cv;
using namespace fiducial;
using namespace opencv_candidate;
using namespace std;

namespace bfs = boost::filesystem;

namespace clutseg {

    void Label::write(cv::FileStorage& fs) const {
        fs << name;
        pose.write(fs);
    }

    void Label::read(const cv::FileNode& fn) {
        name = fn.name();
        pose.read(fn);
    }

    bool LabelSet::onScene(const string & name) const {
        // slow 
        BOOST_FOREACH(const Label & np, labels) {
            if (np.name == name) {
                return true;
            }
        }
        return false;
    }

    int LabelSet::distinctLabelCount() const {
        set<string> d;
        BOOST_FOREACH(const Label & np, labels) {
            d.insert(np.name);
        }
        return d.size();
    }

    vector<PoseRT> LabelSet::posesOf(const string & subject) const {
        vector<PoseRT> ps;
        BOOST_FOREACH(const Label & np, labels) {
            if (np.name == subject) {
                ps.push_back(np.pose);
            }
        }
        return ps;
    }

    void LabelSet::read(const bfs::path & filename) {
        cout << "[GROUND] Reading in " << filename << endl;
        assert_path_exists(filename);
        labels.clear();
        FileStorage fs = FileStorage(filename.string(), FileStorage::READ);
        // iterate over objects
        for (FileNodeIterator n_it = fs.root().begin(); n_it != fs.root().end(); n_it++) {
            // FIXME:
            Label np;
            np.read(*n_it);
            np.pose.estimated = true;
            labels.push_back(np);  
        }
    }

    void LabelSet::write(const bfs::path & filename) const {
        FileStorage fs(filename.string(), FileStorage::WRITE);
        BOOST_FOREACH(const Label & np, labels) {
            // FIXME:
            np.write(fs);
        }
        fs.release();
    } 

    Point projectOrigin(const PoseRT & pose, const opencv_candidate::Camera & camera) {
        Point3d o(0, 0, 0);
        vector<Point3f> op(1);
        vector<Point2f> ip(1);
        op[0] = o;
        projectPoints(Mat(op), pose.rvec, pose.tvec, camera.K, camera.D, ip);
        return ip[0];
    }
    
    void randomizePose(PoseRT & pose, double stddev_t, double stddev_r) {
        mt19937 twister; 
        normal_distribution<> n_t(0, stddev_t);
        variate_generator<mt19937&, normal_distribution<> > noise_t(twister, n_t);
        pose.tvec.at<double>(0, 0) += noise_t();
        pose.tvec.at<double>(1, 0) += noise_t();
        pose.tvec.at<double>(2, 0) += noise_t();
        normal_distribution<> n_r(0, stddev_r);
        variate_generator<mt19937&, normal_distribution<> > noise_r(twister, n_r);
        pose.rvec.at<double>(0, 0) += noise_r();
        pose.rvec.at<double>(1, 0) += noise_r();
        pose.rvec.at<double>(2, 0) += noise_r();
    }

    cv::Mat randomOrientation(double angle) {
        Mat r = Mat::zeros(3, 1, CV_64FC1);
        r.at<double>(0, 0) = rand() - 0.5; 
        r.at<double>(1, 0) = rand() - 0.5; 
        r.at<double>(2, 0) = rand() - 0.5; 
        return angle * (r / norm(r));
    }

    PoseRT poseToPoseRT(const Pose & src) {
        PoseRT dst;
        Mat R;
        eigen2cv(src.t(), dst.tvec);
        eigen2cv(src.r(), R);
        Rodrigues(R, dst.rvec);
        dst.tvec.convertTo(dst.tvec, CV_64F);
        dst.rvec.convertTo(dst.rvec, CV_64F);
        dst.estimated = true;
        return dst;
    }

    Pose poseRtToPose(const PoseRT & src) {
        Pose dst;
        Eigen::Vector3f t;
        cv2eigen(src.tvec, t);
        dst.setT(t);
        Mat R;
        Eigen::Matrix<double, 3, 3> Re;
        Rodrigues(src.rvec, R);
        cv2eigen(R, Re);
        dst.setR(R);
        return dst;
    }

    void writePose(const bfs::path & filename, const PoseRT & pose) {
        FileStorage f;
        f.open(filename.string(), FileStorage::WRITE);
        f << PoseRT::YAML_NODE_NAME;
        pose.write(f);
        f.release();
    }

    void readPose(const bfs::path & filename, PoseRT & dst) {
        assert_path_exists(filename);
        FileStorage f;
        f.open(filename.string(), FileStorage::READ);
        dst.read(f.getFirstTopLevelNode());
        f.release();
    }

    void convertPoseFileToDouble(const boost::filesystem::path & src, const boost::filesystem::path & dst) {
        assert_path_exists(src);
        map<string, PoseRT> poses;
        FileStorage in(src.string(), FileStorage::READ);
        for (FileNodeIterator n_it = in.root().begin(); n_it != in.root().end(); n_it++) {
            Pose p;
            p.read(*n_it);
            string n = (*n_it).name();
            // why doesn't this work?
            // n = n_it->name();
            poses[n] = poseToPoseRT(p);
        }
        in.release();
        FileStorage out(dst.string(), FileStorage::WRITE);
        // preprocessor does not allow definition with < > in BOOST_FOREACH
        typedef pair<string, PoseRT> pair_t;
        BOOST_FOREACH(pair_t e, poses) {
            out << e.first;
            e.second.write(out);
        }
        out.release();
    }

    void modelToView(const Mat & mvtrans, const Mat & mvrot, const Mat & mpt, Mat & vpt) {
        vpt = mvrot * mpt + mvtrans;
    }

    void modelToView(const PoseRT & pose, const Point3d & mpt, Point3d & vpt) {
        vector<Point3d> op(1, mpt);
        Mat mop(op);
        Mat R;
        Rodrigues(pose.rvec, R);
        transform(mop, mop, R);
        op[0] += pose.tvec.at<Point3d>();
        vpt = op[0];
    }

    opencv_candidate::PoseRT translatePose(const opencv_candidate::PoseRT & src, const Mat & model_tvec) {
        PoseRT dst;
        src.rvec.convertTo(dst.rvec, CV_64FC1);
        Mat mvrot;
        Rodrigues(dst.rvec, mvrot);
        Mat st;
        src.tvec.convertTo(st, CV_64FC1);
        modelToView(st, mvrot, model_tvec, dst.tvec);
        dst.estimated = true;
        return dst;
    }

    opencv_candidate::PoseRT rotatePose(const opencv_candidate::PoseRT & src, const Mat & model_rvec) {
        Mat sr, mr;
        src.rvec.convertTo(sr, CV_64FC1);
        model_rvec.convertTo(mr, CV_64FC1);

        Mat P;
        Rodrigues(sr, P);
        Mat D;
        Rodrigues(mr, D);
        PoseRT dst;
        Rodrigues(P * D, dst.rvec);
        dst.tvec = src.tvec.clone();
        dst.estimated = true;
        return dst;
    }

    double angleBetweenVectors(const cv::Mat & u, const cv::Mat & v) {
        double c = u.dot(v) / (norm(u) * norm(v));
        // The calculation of cosine 'c' above is not always completely exact.
        // It is possible that c > 1 due to inexactness of floating-point
        // arithmetic. If min is omitted angleBetween(p, p) will produce NAN
        // instead of zero.
        return acos(min(1.0, c));
    }

    cv::Mat diffRotation(const cv::Mat & P, const cv::Mat & Q) {
        return P.inv() * Q;
    }


    double angle_between(const opencv_candidate::PoseRT & p,
                                    const opencv_candidate::PoseRT & q) {
        // See also comment on dist_between for information about type
        // conversion.
        Mat pr, qr;
        p.rvec.convertTo(pr, CV_64FC1);
        q.rvec.convertTo(qr, CV_64FC1);
        Mat P, Q;
        Rodrigues(pr, P);
        Rodrigues(qr, Q);
        Mat d;
        Rodrigues(P.inv() * Q, d);
        return norm(d);
    }

    double dist_between(const opencv_candidate::PoseRT & p,
                                const opencv_candidate::PoseRT & q) {
        // There is a type confusion issue with opencv_candidate::PoseRT and
        // opencv_candidate::Pose which are not compliant with used Matrix
        // types, therefore we cannot rely on p, q having members of the same
        // data type.
        Mat pt, qt;
        p.tvec.convertTo(pt, CV_64FC1);
        q.tvec.convertTo(qt, CV_64FC1);
        return norm(qt - pt);
    }

}

