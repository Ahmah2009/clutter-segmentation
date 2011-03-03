#ifndef _PASDetection
#define _PASDetection

#include "opencv/cv.h"
#include "opencv/cvaux.h"
#include "opencv/highgui.h"

#include <set>
#include <utility>
#include <sstream>

#define _USE_MATH_DEFINES
#include <math.h>

//#define OPENCV_PAS_BRANCH

#include <chamfer_matching/chamfer_matching.h>
using namespace cv;

struct Segment
{
    Point back;
    Point front;
    Point2d direction;

    std::set<int> neighborhood;

    bool isBackSegment;
    bool isFrontSegment;

    Segment( Point _back, Point _front ) : back( _back ), front( _front ), isBackSegment( false ), isFrontSegment( false ) {}

    bool isEndSegment()
    {
        return ( isBackSegment || isFrontSegment );
    }

    float getOrientation() const;
    float getNorm() const;
};

float norm(const Segment &segment);

struct PAS
{
    struct PASDescriptor
    {
        float orientation1;
        float orientation2;
        float length1;
        float length2;
        Point2f rVector;
        Point2f intersectPoint;

        PASDescriptor( ) {}
        PASDescriptor( const float *PASDescriptor )
        {
            orientation1 = PASDescriptor[1];
            orientation2 = PASDescriptor[2];
            length1 = PASDescriptor[3];
            length2 = PASDescriptor[4];
            rVector.x = PASDescriptor[5];
            rVector.y = PASDescriptor[6];
        }

        operator Mat() const
        {
            Mat result( 1, descriptorLength, CV_32FC1 );

            Mat_<float>::iterator it = result.begin<float>();

            *(it++) = 2; //number of segments in feature;

            *(it++) = orientation1;
            *(it++) = orientation2;
            *(it++) = length1;
            *(it++) = length2;
            *(it++) = rVector.x;
            *(it++) = rVector.y;

            return result;
        }

        static float dtheta( float orientation1, float orientation2 )
        {
            float diff = fabs( orientation1 - orientation2 );
            return ( diff <= M_PI_2 ? diff : M_PI - diff );
        }

        static float dissimilarity( const PASDescriptor &desc1, const PASDescriptor &desc2 )
        {
            //float wr = 4;
            //float wth = 2;
            float wr = 0.03;
            float wth = 1;
            float result = 0;
            result += wth*( dtheta( desc1.orientation1, desc2.orientation1 ) + dtheta( desc1.orientation2, desc2.orientation2 ) );
            result += fabs( log( desc1.length1 / desc2.length1 ) ) + fabs( log( desc1.length2 / desc2.length2 ) );
            result += wr*( sqrt( pow( desc1.rVector.x - desc2.rVector.x, 2 ) + pow( desc1.rVector.y-desc2.rVector.y, 2 ) ) );
            return result;
        }

        static const int descriptorLength = 7;
    };


    Point2f location;
    float scale;
    float strength;
    PASDescriptor descriptor;

    int index1;
    int index2;


    operator Mat() const
    {
        return Mat( descriptor );
    }
};

//float calcDissimilarity ( const Mat &a, const Mat &b );

class PASDescriptor : public DescriptorExtractor
//class PASDescriptor : public features_2d::DescriptorExtractor
{
public:
    virtual void compute( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors ) const;
};

class PASDetector : public FeatureDetector
//class PASDetector : public features_2d::FeatureDetector
{
protected:
    virtual void detectImpl( const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints ) const;
};

class ContourSegmentNetwork
{
public:
    struct HoughParams
    {
        static const int HOUGH_NX=20;
        static const int HOUGH_NY=20;
        static const int HOUGH_NSCALE=3;
        static double GET_MIN_LG_SCALE() { return -0.5; }
        static double GET_MAX_LG_SCALE() { return 0.5; }
        static float GET_HOUGH_DISSIMILARITY_THRESHOLD() { return 3.; }
        static float GET_HOUGH_THRESHOLD_NORMALIZER() { return 0.5/3.; }


        int nx;
        int ny;
        int nscale;
        double minLgScale;
        double maxLgScale;
        float dissimilarityThreshold;
        float houghThresholdNormalizer;

        HoughParams( int _nx=HOUGH_NX, int _ny=HOUGH_NY, int _nscale=HOUGH_NSCALE,
                     double _minLgScale=GET_MIN_LG_SCALE(), double _maxLgScale=GET_MAX_LG_SCALE(),
                     float _dissimilarityThreshold=GET_HOUGH_DISSIMILARITY_THRESHOLD(),
                     float _houghThresholdNormalizer=GET_HOUGH_THRESHOLD_NORMALIZER() ) :
                         nx(_nx), ny(_ny), nscale(_nscale),
                         minLgScale(_minLgScale), maxLgScale(_maxLgScale),
                         dissimilarityThreshold(_dissimilarityThreshold),
                         houghThresholdNormalizer(_houghThresholdNormalizer) {}
    };

    //ContourSegmentNetwork();

    void calcContourSegmentNetwork( const Mat &image, bool isObject = false );


    void detectByHoughVoting( const ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects, bool visualizeHT, Mat &HTVisualization, HoughParams houghParams=HoughParams() );
    Mat visualizeHoughTransform(const ContourSegmentNetwork &testCSN, const MatND &houghSpace, Point2d maxTranslation, const HoughParams &houghParams, const vector< std::pair<Rect, double> > &boundingRects) const;

    void detectBySlidingWindow( const Mat &codebook, double dissimilarityThreshold, const MatND& integralHistogram, const ContourSegmentNetwork &testCSN, const Mat &idf, vector< std::pair<Rect, double> > &boundingRects, bool visualize, Mat &visualization ) const;
    void detectByChamferMatching( ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects, bool visualize, Mat &visualization ) const;
    Mat visualizeBoundingRects( ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects ) const;

    int drawSegment( Mat &image, int i=-1, bool isDrawNeighborhood=true ) const;
    Mat drawAllSegments() const;
    Mat drawAllApproxCurves() const;
    int drawPAS( Mat &image, int i=-1 ) const;
    Mat drawAllPASes() const;

    vector<KeyPoint> PASesToKeypoints () const;

    int getRows() const { return baseImage.rows; }
    int getCols() const { return baseImage.cols; }
    double getPerimeter() const { return perimeter; }
    vector<PAS> PASes;

    //public for Debug
    void calcCurves( const vector<vector<Point> > &contours, int contourIdx, vector<vector<Point> >&curves, float minContourLength=GET_MIN_CONTOUR_LENGTH(), float minCurveLength=GET_MIN_CURVE_LENGTH(), float minDistToEdgel = GET_MIN_DIST_TO_EDGEL() );
    //Mat filterEdgesByLength( Mat edges, float minContourLength = GET_MIN_CONTOUR_LENGTH() ) const;
    Mat filterEdgesByLength( Mat edges, float minContourLength = GET_MIN_CHAMFER_CONTOUR_LENGTH() ) const;
    Mat calcEdges( const Mat &image, double threshold1=GET_CANNY_THRESHOLD_1(), double threshold2=GET_CANNY_THRESHOLD_2() ) const;
    Mat calcObjectEdges( const Mat &image, double threshold1=GET_CANNY_THRESHOLD_1(), double threshold2=GET_CANNY_THRESHOLD_2() ) const;
    static Rect calcObjectBoundingBox( const Mat &edges );
private:
    void calcSegmentedContours( Mat &edges, float minContourLength=GET_MIN_CONTOUR_LENGTH(), double epsilon=GET_APPROX_POLY_DP_EPSILON() );
    void calcContours( );
    void bridgeSegments();
    void linkSegments();
    void calcPASes();
    void filterPASes();
    void calcPerimeter();

    float calcHoughThreshold(const HoughParams &houghParams) const;
    void filterByDistanceTransform();

    bool segmentTrapeziumTest( Point2f point, Point2f direction, const Segment &segment2, float a=GET_TRAPEZIUM_A(), float b=GET_TRAPEZIUM_B(), float h=GET_TRAPEZIUM_H());
    bool isSegmentsLinked( const Segment &segment1, const Segment &segment2 );
    bool isSegmentsBridged( const Segment &segment1, const Segment &segment2, float epsilon=GET_BRIDGE_EPSILON() );
    void calcProjection( Point2f direction, Point2f point,  Point2f &a, Point2f &b, float &min, float &max);


    bool isSegmentsNotNull();
    vector<Segment> segments;
    vector<int> segmentsIndices;


    //TODO: for debugging, remove
    vector<vector<Point> > approxCurves;
    vector<vector<Point> > approxContours;

    Mat baseImage;
    //Mat baseEdges;
    //Rect boundingBox;

    Mat derivativeX;
    Mat derivativeY;
    Mat derivatives;

    double perimeter;

    //static double GET_CANNY_THRESHOLD_1() { return 150; }
    //static double GET_CANNY_THRESHOLD_2() { return 100; }

    static double GET_CANNY_THRESHOLD_1() { return 120; }
    static double GET_CANNY_THRESHOLD_2() { return 80; }

    static float GET_TRAPEZIUM_A() { return 2; }
    static float GET_TRAPEZIUM_B() { return 6; }
    static float GET_TRAPEZIUM_H() { return 12; }

    static float GET_MIN_CHAMFER_CONTOUR_LENGTH() { return 64; }
    static float GET_MIN_CONTOUR_LENGTH() { return 64; }
    static float GET_MIN_CURVE_LENGTH() { return 8; }

    static float GET_MIN_DIST_TO_EDGEL() { return 1.01; }


    static float GET_APPROX_POLY_DP_EPSILON() { return 2; }
    static float GET_BRIDGE_EPSILON() { return 0.05; }

    friend void constructCodebook( const vector<ContourSegmentNetwork*> &csns, Mat &codebook, double dissimilarityThreshold );
    friend MatND computeIntegralHistogram( const Mat &codebook, const ContourSegmentNetwork &testCSN, double dissimilarityThreshold );
    friend void calcIDF( const vector<ContourSegmentNetwork*> &csns, const Mat &codebook, Mat &idf, double dissimilarityThreshold );
};

template<class T>
struct CV_EXPORTS PASDissimilarity
{
    typedef T ValueType;
    typedef typename Accumulator<T>::Type ResultType;

    ResultType operator()( const T* a, const T* b, int size ) const
    {
        //if (a[0] == 2 && b[0] == 2)
            return PAS::PASDescriptor::dissimilarity( a, b );
        if (a[0] == 1 && b[0] == 1)
        {
            float result = 0;
            float wth = 2;
            result += fabs( log( a[1] / b[1] ) );
            result += wth*( PAS::PASDescriptor::dtheta( a[2], b[2] ) );

            result *= 2;
            return result;
        }
        return std::numeric_limits<float>::max();
    }
};


//void constructCodebook( const vector<PAS> &pases, Mat &codebook );
void visualizeCodebook( const Mat &codebook, const Mat &idf );
void sortCodebook( Mat &codebook, Mat &idf );


#endif
