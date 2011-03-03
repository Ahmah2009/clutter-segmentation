#include "textured_object_detection/PASDetection.hpp"

#include <math.h>
#include <list>

float Segment::getOrientation() const
{
    Point2f vec = front - back;
    const float epsilon = 1e-6;
    if( fabs(vec.x) > epsilon)
        return atan( vec.y / vec.x );
    else
        return ( vec.y > 0 ? M_PI_2 : -M_PI_2 );
}

float Segment::getNorm() const
{
    return norm( front - back );
}

void ContourSegmentNetwork::calcContourSegmentNetwork( const Mat &image, bool isObject )
{
    baseImage = image.clone();
    Sobel( baseImage, derivativeX, CV_32FC1, 1, 0 );
    Sobel( baseImage, derivativeY, CV_32FC1, 0, 1 );
    derivatives = abs( derivativeX ) + abs( derivativeY );

    //imshow("base", baseImage);
    //imshow("x", derivativeX);
    //imshow("y", derivativeY);
    //imshow("all", derivatives);
    //waitKey( 0 );

    Mat edges;
    //if( isObject)
    {
//    	edges = calcObjectEdges( image );
    }
    //else
    {
    	edges = calcEdges( image );
    }

    //imshow("edges", baseEdges);
    //waitKey();

    calcSegmentedContours( edges );

    linkSegments();

    bridgeSegments();
    calcPASes();

    filterPASes();
    calcPerimeter();
}

bool ContourSegmentNetwork::isSegmentsNotNull()
{
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        if( segmentsIndices[i] == -1 )
            continue;
        Segment segment = segments[ segmentsIndices[i] ];
        if( segment.front == segment.back )
            return false;
    }
    return true;
}


float norm(const Segment &segment)
{
    return segment.getNorm();
}

void ContourSegmentNetwork::calcPerimeter( )
{
    perimeter = 0;
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        int idx = segmentsIndices[i];
        if( idx != -1 )
        {
            perimeter += segments[ idx ].getNorm();
        }
    }
}

void ContourSegmentNetwork::calcPASes( )
{
    PASes.clear();
    //TODO: check double edges between segments
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        if( segmentsIndices[i] < 0 )
            continue;
        Segment segment = segments[ segmentsIndices[i] ];

        for( std::set<int>::iterator p=segment.neighborhood.begin(); p != segment.neighborhood.end(); p++ )
        {
            if( i > *p )
                continue;
            Segment segment1 = segment;
            Segment segment2 = segments[ segmentsIndices[ *p ] ];
            PAS pas;
            Point2f center1 = ( segment1.front + segment1.back ) * 0.5f;
            Point2f center2 = ( segment2.front + segment2.back ) * 0.5f;
            pas.location = ( center1 + center2 ) * 0.5f;
            pas.index1 = segmentsIndices[ i ];
            pas.index2 = segmentsIndices[ *p ];

            bool isInverse = norm( center2 - pas.location ) < norm( center1 - pas.location);
            isInverse = isInverse || center1.x > center2.x || ( center1.x == center2.x && center1.y > center2.y );
            if( isInverse )
            {
                pas.index1 = segmentsIndices[ *p ];
                pas.index2 = segmentsIndices[ i ];

                segment1 = segment2;
                segment2 = segment;
                center1 = ( segment1.front + segment1.back ) * 0.5f;
                center2 = ( segment2.front + segment2.back ) * 0.5f;
            }

            pas.scale = norm( center1 - center2 );

            //if (for example) segments is crossed then norm( center1 - center2 ) may be is null. So it is unreliable for norm determination.
            //pas.scale = segment1.getNorm() + segment2.getNorm();

            /*
            if( pas.scale < 1e-6 )
            {
                static int k = 0;
                printf("(%f, %f), (%f, %f)\n", center1.x, center1.y, center2.x, center2.y);
                int isLinked = isSegmentsLinked( segment1, segment2 );
                int isBridged = isSegmentsBridged( segment1, segment2 );
                printf("Linked: %d, Bridged: %d\n", isLinked, isBridged);
                printf("Segment1: (%d, %d), (%d, %d)\n", segment1.back.x, segment1.back.y, segment1.front.x, segment1.front.y);
                printf("Segment2: (%d, %d), (%d, %d)\n", segment2.back.x, segment2.back.y, segment2.front.x, segment2.front.y);

                if( k==0 )
                {
                    float a = 2;
                    float b = 6;
                    float h = 10;
                    //Point2f direction = segment1.back - segment1.front;
                    Point2f direction = segment1.front - segment1.back;
                    printf("Direction: %f, %f\n", direction.x, direction.y );
                    direction *= 1.f / norm( direction );
                    printf("Direction: %f, %f\n", direction.x, direction.y );
                    Point2f normal = Point2f( -direction.y, direction.x );
                    printf("Normal: %f, %f\n", normal.x, normal.y );

                    //Point2f direction2 = segment2.front - segment2.back;
                    //Point2f normal2 = Point2f( -direction2.y, direction2.x );
                    //float c2 = -normal2.dot( segment2.back );

                    Mat trapezium = Mat( 1, 4, CV_32FC2 );

                    //Point2f point = segment1.back;
                    Point2f point = segment1.front;
                    trapezium.at<Point2f>( 0, 0 ) = point + normal * ( a/2.f );
                    trapezium.at<Point2f>( 0, 1 ) = point + direction*h + normal * ( b/2.f );
                    trapezium.at<Point2f>( 0, 2 ) = point + direction*h - normal * ( b/2.f );
                    trapezium.at<Point2f>( 0, 3 ) = point - normal * ( a/2.f );
                    printf("Trapezium:\n");
                    printf("%f, %f\n", trapezium.at<Point2f>( 0, 0 ).x, trapezium.at<Point2f>( 0, 0 ).y );
                    printf("%f, %f\n", trapezium.at<Point2f>( 0, 1 ).x, trapezium.at<Point2f>( 0, 1 ).y );
                    printf("%f, %f\n", trapezium.at<Point2f>( 0, 2 ).x, trapezium.at<Point2f>( 0, 2 ).y );
                    printf("%f, %f\n", trapezium.at<Point2f>( 0, 3 ).x, trapezium.at<Point2f>( 0, 3 ).y );

                }

                k=1;
            }
            */
            pas.scale = pas.scale < 0.1 ? 0.1 : pas.scale;
            PAS::PASDescriptor descriptor;
            descriptor.orientation1 = segment1.getOrientation();
            descriptor.orientation2 = segment2.getOrientation();
            //descriptor.length1 = segment1.getNorm() / pas.scale;
            //descriptor.length2 = segment2.getNorm() / pas.scale;
            //descriptor.rVector = (center2 - center1) * ( 1.f / pas.scale );
            descriptor.length1 = segment1.getNorm();
            descriptor.length2 = segment2.getNorm();
            descriptor.rVector = (center2 - center1);

            pas.strength = 1;

            double s1 = 0;
            double s2 = 0;
            if( 0 <= center1.x && center1.x < baseImage.cols && 0 <= center1.y && center1.y < baseImage.rows )
                if( 0 <= center2.x && center2.x < baseImage.cols && 0 <= center2.y && center2.y < baseImage.rows )
                {
                    //printf( "%f, %f\n", center1.x, center1.y );
                    //printf( "%f, %f\n", center2.x, center2.y );
                    s1 =  derivatives.at<float>( center1.y, center1.x );
                    s2 =  derivatives.at<float>( center2.y, center2.x );
                }


            if( descriptor.length1 + descriptor.length2 > 1e-6)
                pas.strength = ( descriptor.length1 * s1 + descriptor.length2 * s2 ) / ( descriptor.length1 + descriptor.length2 );
            else
                pas.strength = 0;

/*
            float a1 = -segment1.direction.y;
            float b1 = segment1.direction.x;
            float c1 = -Point2f( a1, b1).dot( segment1.back );
            float a2 = -segment2.direction.y;
            float b2 = segment2.direction.x;
            float c2 = -Point2f( a2, b2).dot( segment2.back );
            */

            Point2f dir = segment1.front - segment1.back;
            float a1 = -dir.y;
            float b1 = dir.x;
            float c1 = -Point2f( a1, b1).dot( segment1.back );
            dir = segment2.front - segment2.back;
            float a2 = -dir.y;
            float b2 = dir.x;
            float c2 = -Point2f( a2, b2).dot( segment2.back );

            float det = a1*b2 - a2*b1;

            float minDet = 1e-6;
            det = fabs(det) < minDet ? minDet : det;
            descriptor.intersectPoint = Point2f( -( c1*b2 - c2*b1 ) / det, -( a1*c2 - a2*c1 ) / det );

            pas.descriptor = descriptor;
            PASes.push_back( pas );
        }
    }
}

Mat ContourSegmentNetwork::calcEdges( const Mat &image, double threshold1, double threshold2 ) const
{
    Mat edges = Mat( baseImage.rows, baseImage.cols, CV_8UC1, Scalar( 0 ) );
    Canny( image, edges, threshold1, threshold2 );
    return edges;
}

void ContourSegmentNetwork::calcCurves( const vector<vector<Point> > &contours, int contourIdx, vector<vector<Point> >&curves, float minContourLength, float minCurveLength, float minDistToEdgel )
{
    curves.clear();

    int basePoints = 5;
    int curveIndex = -1;
    bool isPreviousOk = false;
    for( size_t i=0;i<contours[contourIdx].size();i++ )
    {
        bool ok = true;


        //check points on the same contour
        for( int k=0;k<=curveIndex;k++ )
        {
            //if the same curve don't check last points
            int max = (k == curveIndex) ? curves[ curveIndex ].size() - basePoints : curves[k].size();

            //j=1 to allow closed curves
            for( int j=1;j<max;j++ )
            {
                 if( norm( contours[contourIdx][i] - curves[ k ][j] ) < minDistToEdgel )
                 {
                     ok = false;
                     break;
                 }
            }
        }

        //check points on other contours
        for( int ctr=0;ctr<contourIdx;ctr++ )
        {
            double length = arcLength( Mat( contours[ctr] ), false );
            if( length < minContourLength )
                continue;

            if( ok )
            {
                for( int point=0;point<contours[ctr].size();point++ )
                {
                    if( norm( contours[contourIdx][i] - contours[ctr][point] ) < minDistToEdgel )
                    {
                        ok = false;
                        break;
                    }
                }
            }
        }

        if( ok )
        {
            if( isPreviousOk )
                curves[ curveIndex ].push_back( contours[contourIdx][i] );
            else
            {
                curves.push_back( vector<Point> () );
                curveIndex++;
                curves[ curveIndex ].push_back( contours[contourIdx][i] );
            }
        }
        isPreviousOk = ok;
    }

    for( vector<vector<Point> >::iterator it=curves.begin();it!=curves.end(); )
    {
        double length = arcLength( Mat( *it ), false );
        if( length < minCurveLength )
        {
            it = curves.erase( it );
        }
        else
        {
            it++;
        }
    }
}

int getNextContour( vector<Vec4i> &hierarchy, int i )
{
    if( hierarchy[i][0] >= 0)
    {
        return hierarchy[i][0];
    }
    if( hierarchy[i][3] < 0)
        return -1;

    int parent = hierarchy[i][3];
    if( hierarchy[parent][0] >= 0)
    {
        return hierarchy[parent][0];
    }
    return -1;
}

Mat ContourSegmentNetwork::filterEdgesByLength( Mat edges, float minContourLength ) const
{
    vector<vector<Point> > contours;
    findContours( edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point() );
    Mat result( edges.rows, edges.cols, edges.type(), Scalar( 0 ) );

    for( size_t contourIdx=0;contourIdx<contours.size();contourIdx++ )
    {
        double length = arcLength( Mat( contours[contourIdx] ), false );
        if( length < minContourLength )
            continue;
        for( size_t i=0;i<contours[contourIdx].size();i++ )
            result.at<unsigned char>( contours[contourIdx][i].y, contours[contourIdx][i].x ) = 255;
    }
    return result;
}

void ContourSegmentNetwork::calcSegmentedContours( Mat &edges, float minContourLength, double epsilon )
{
    vector<vector<Point> > contours;
    //vector<Vec4i> hierarchy;
    //findContours( edges, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point() );
    //TODO: solve segments duplication due to overlapping contours
    findContours( edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point() );
    approxContours.clear();


//    for( int contourIdx=0;contourIdx>=0;contourIdx=getNextContour( hierarchy, contourIdx) )
//    {
//        if( hierarchy[contourIdx][2] >= 0)
//        {
//            contourIdx = hierarchy[contourIdx][2];
//        }

    for( size_t contourIdx=0;contourIdx<contours.size();contourIdx++ )
    {
        double length = arcLength( Mat( contours[contourIdx] ), false );
        if( length < minContourLength )
            continue;
        vector<vector<Point> > curves;
        calcCurves( contours, contourIdx, curves );

        for( size_t k=0;k<curves.size();k++ )
        {
            //if( curves[k].size() == 1)
                //continue;
            vector<Point> approxContour;
            approxPolyDP( Mat( curves[k] ), approxContour, epsilon, false );
            approxContours.push_back( approxContour );
            approxCurves.push_back( curves[k] );

            int prev = -1;
            for( size_t j=0;j<approxContour.size()-1;j++)
            {
                Segment segment = Segment( approxContour[j], approxContour[j+1] );
                if( norm(segment) <= 0.5 )
                {
                    printf("Yes, it is possible: segments is null!\n");
                    continue;
                }

                segment.direction = segment.front - segment.back;
                segment.direction *= 1.f / norm( segment.direction );
                segment.isBackSegment = ( j == 0 );
                segment.isFrontSegment = ( j == approxContour.size()-2 );

                if( prev > 0 )
                {
                    segment.neighborhood.insert( prev );
                    segments[ segmentsIndices[ prev ] ].neighborhood.insert( segments.size() );
                }

                prev = segments.size();
                segmentsIndices.push_back( segments.size() );
                segments.push_back( segment );

                assert( segment.front != segment.back );
            }
        }
    }
}

bool ContourSegmentNetwork::segmentTrapeziumTest( Point2f point, Point2f direction, const Segment &segment2, float a, float b, float h )
{
    /*
    Point2d direction = dir;
    direction *= 1. / norm( direction );
    Point2d normal = Point2d( -direction.y, direction.x );

    Point2d direction2 = segment2.front - segment2.back;
    Point2d normal2 = Point2d( -direction2.y, direction2.x );
    double c2 = -normal2.dot( segment2.back );

    Mat trapezium = Mat( 1, 4, CV_32FC2 );

    Point2d point = p;
    trapezium.at<Point2f>( 0, 0 ) = point + normal * ( a/2. );
    trapezium.at<Point2f>( 0, 1 ) = point + direction*h + normal * ( b/2. );
    trapezium.at<Point2f>( 0, 2 ) = point + direction*h - normal * ( b/2. );
    trapezium.at<Point2f>( 0, 3 ) = point - normal * ( a/2. );

    if( pointPolygonTest( trapezium, segment2.back, false ) >= 0 || pointPolygonTest( trapezium, segment2.front, false ) >= 0 )
        return true;

    for( size_t i=0;i<4;i++ )
    {
        Point2d a = trapezium.at<Point2f>( 0, i % 4 );
        Point2d b = trapezium.at<Point2f>( 0, (i+1) % 4 );
        Point2d dir = b - a;
        Point2d norm = Point2d( -dir.y, dir.x );
        float c = -norm.dot( a );

        bool flag1 = ( ( normal2.dot( a ) + c2 ) * ( normal2.dot( b ) + c2 ) ) <= 0;
        bool flag2 = ( ( norm.dot( segment2.back ) + c ) *( norm.dot( segment2.front ) + c ) ) <= 0;
        if( flag1 && flag2 )
            return true;
    }
    return false;
*/

    direction *= 1.f / norm( direction );
    Point2f normal = Point2f( -direction.y, direction.x );

    Point2f direction2 = segment2.front - segment2.back;
    Point2f normal2 = Point2f( -direction2.y, direction2.x );
    float c2 = -normal2.dot( segment2.back );

    Mat trapezium = Mat( 1, 4, CV_32FC2 );

    trapezium.at<Point2f>( 0, 0 ) = point + normal * ( a/2.f );
    trapezium.at<Point2f>( 0, 1 ) = point + direction*h + normal * ( b/2.f );
    trapezium.at<Point2f>( 0, 2 ) = point + direction*h - normal * ( b/2.f );
    trapezium.at<Point2f>( 0, 3 ) = point - normal * ( a/2.f );

    if( pointPolygonTest( trapezium, segment2.back, false ) >= 0 || pointPolygonTest( trapezium, segment2.front, false ) >= 0 )
        return true;

    for( size_t i=0;i<4;i++ )
    {
        Point2f a = trapezium.at<Point2f>( 0, i % 4 );
        Point2f b = trapezium.at<Point2f>( 0, (i+1) % 4 );
        Point2f dir = b - a;
        Point2f norm = Point2f( -dir.y, dir.x );
        float c = -norm.dot( a );

        bool flag1 = ( ( normal2.dot( a ) + c2 ) * ( normal2.dot( b ) + c2 ) ) <= 0;
        bool flag2 = ( ( norm.dot( segment2.back ) + c ) *( norm.dot( segment2.front ) + c ) ) <= 0;
        if( flag1 && flag2 )
            return true;
    }
    return false;
}

bool ContourSegmentNetwork::isSegmentsLinked( const Segment &segment1, const Segment &segment2 )
{
    //assert( segment1.isBackSegment || segment1.isFrontSegment );
    if( segment1.back == segment2.back || segment1.back == segment2.front || segment1.front == segment2.back || segment1.front == segment2.front )
        return true;

    Point2f point;
    Point2f direction;
    if( segment1.isBackSegment )
    {
        point = segment1.back;
        direction = segment1.back - segment1.front;
        if( segmentTrapeziumTest( point, direction, segment2 ) )
            return true;
    }

    if( segment1.isFrontSegment )
    {
        point = segment1.front;
        direction = segment1.front - segment1.back;
        if( segmentTrapeziumTest( point, direction, segment2 ) )
            return true;
    }

    if( segment2.isBackSegment )
    {
        point = segment2.back;
        direction = segment2.back - segment2.front;
        if( segmentTrapeziumTest( point, direction, segment1 ) )
            return true;
    }

    if( segment2.isFrontSegment )
    {
        point = segment2.front;
        direction = segment2.front - segment2.back;
        if( segmentTrapeziumTest( point, direction, segment1 ) )
            return true;
    }

    return false;
}

bool ContourSegmentNetwork::isSegmentsBridged( const Segment &segment1, const Segment &segment2, float epsilon )
{
   return fabs( segment1.direction.dot( segment2.direction ) ) > 1 - epsilon;
}

void ContourSegmentNetwork::calcProjection( Point2f direction, Point2f point, Point2f &a, Point2f &b, float &min, float &max)
{
    float dot = direction.dot( point );
    if( dot < min )
    {
        min = dot;
        a = point;
    }
    else if( dot > max )
    {
        max = dot;
        b = point;
    }
}

void ContourSegmentNetwork::bridgeSegments()
{
    bool isVisualize = false;

    bool isBridged = true;
    while( isBridged )
    {
        isBridged = false;
        for( size_t i=0;i<segmentsIndices.size();i++ )
        {
            if( segmentsIndices[i] < 0 )
                continue;
            Segment &segment = segments[ segmentsIndices[i] ];

            if( !segment.isEndSegment() )
                continue;
            for( size_t j=0;j<segmentsIndices.size();j++ )
            {
                if( i==j )
                    continue;
                if( segmentsIndices[j] < 0 )
                    continue;

                Segment &segment2 = segments[ segmentsIndices[j] ];

                if( !isSegmentsLinked( segment, segment2 ) )
                    continue;

                if( isSegmentsBridged( segment, segment2 ) )
                {
                    if( isVisualize )
                    {
                        Mat csnImage = drawAllSegments();
                        Mat csnImageColor;
                        cvtColor( csnImage, csnImageColor, CV_GRAY2RGB );
                        line( csnImageColor, segment.back, segment.front, Scalar( 255, 0, 0 ) );
                        line( csnImageColor, segment2.back, segment2.front, Scalar( 0, 0, 255 ) );
                        imshow( "csn", csnImageColor );
                        waitKey( 0 );
                    }



                    isBridged = true;
                    Point2f direction;
                    if( segment.direction.dot( segment2.direction) > 0 )
                        direction = segment.direction + segment2.direction;
                    else
                        direction = segment.direction - segment2.direction;
                    direction *= 1.f / norm( direction );
                    float min, max;
                    max = min = direction.dot( segment.front );
                    Point2f a, b;
                    a = b = segment.front;

                    calcProjection( direction, segment.back, a, b, min, max );
                    calcProjection( direction, segment2.front, a, b, min, max );
                    calcProjection( direction, segment2.back, a, b, min, max );

                    /*
                    Point2f normal = Point2f( -direction.y, direction.x );
                    float minNormal, maxNormal;
                    maxNormal = minNormal = normal.dot( segment.front );
                    Point2f aNormal, bNormal;
                    aNormal = bNormal = segment.front;
                    calcProjection( normal, segment.back, aNormal, bNormal, minNormal, maxNormal );
                    calcProjection( normal, segment2.front, aNormal, bNormal, minNormal, maxNormal );
                    calcProjection( normal, segment2.back, aNormal, bNormal, minNormal, maxNormal );
                    */

                    segment.back = a;
                    segment.front = b;
                    segment.direction = direction;
                    //segment.direction = b - a;
                    //segment.direction *= 1.f / norm( direction );

                    //printf( "%d %d\n", segment2.neighborhood.size(), segment.neighborhood.size() );
                    std::copy( segment2.neighborhood.begin(), segment2.neighborhood.end(), std::inserter(segment.neighborhood, segment.neighborhood.begin()) );
                    segmentsIndices[j] = -1;
                    for( std::set<int>::iterator neighbor = segment2.neighborhood.begin(); neighbor != segment2.neighborhood.end(); neighbor++ )
                    {
                        segments[ *neighbor ].neighborhood.erase( j );
                        segments[ *neighbor ].neighborhood.insert( i );
                    }
                    segment.neighborhood.erase( i );



                    if( isVisualize )
                    {
                        Mat csnImage = drawAllSegments();
                        imshow( "csn", csnImage );
                        waitKey( 0 );
                    }
                }
            }
        }
    }
}

void ContourSegmentNetwork::linkSegments()
{
    //TODO: check duplication of data and neighborhoods
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        Segment &segment = segments[ segmentsIndices[i] ];
        if( !segment.isEndSegment() )
            continue;
        for( size_t j=0;j<segmentsIndices.size();j++ )
        {
            if( i == j)
                continue;

            Segment &segment2 = segments[ segmentsIndices[j] ];
            if( !isSegmentsLinked( segment, segment2 ) )
                continue;

            segment2.neighborhood.insert( i );
            segment.neighborhood.insert( j );
        }
    }
}

void ContourSegmentNetwork::filterPASes()
{
    //TODO: remove code duplication with object.cpp
    Mat circleMask(baseImage.size(), CV_8UC1);
    int size=1;
    for(vector<PAS>::iterator it = PASes.begin(); it != PASes.end(); )
    {
        //Point2f pt = it->descriptor.intersectPoint;
        Point pt = it->descriptor.intersectPoint;
        if( pt.x < 0 || pt.x >= baseImage.cols)
        {
            it = PASes.erase(it);
            continue;
        }
        if( pt.y < 0 || pt.y >= baseImage.rows)
        {
            it = PASes.erase(it);
            continue;
        }

        circleMask.setTo(Scalar::all(0));
        circle(circleMask, pt, 2*size+1, CV_RGB(255, 255, 255), -1);
        if( countNonZero( baseImage==0 & circleMask ) )
        {
            it = PASes.erase(it);
        }
        else
        {
          ++it;
        }
    }
}

float ContourSegmentNetwork::calcHoughThreshold(const HoughParams &houghParams) const
{
    //return PASes.size() * houghParams.houghThresholdNormalizer;
    //return getPerimeter() * houghParams.houghThresholdNormalizer;
    return PASes.size() * houghParams.houghThresholdNormalizer;
}

void ContourSegmentNetwork::filterByDistanceTransform()
{
    const float maxDistance = 24;

    Mat distances;
    distanceTransform( baseImage, distances, CV_DIST_L1, CV_DIST_MASK_PRECISE );
    for (vector<PAS>::iterator it = PASes.begin(); it != PASes.end();)
    {
      Point2f pt = it->descriptor.intersectPoint;
      //printf( "(%d, %d): %f\n", (int)pt.y, (int)pt.x, distances.at<float> ((int)pt.y, (int)pt.x));
      if (distances.at<float> ((int)pt.y, (int)pt.x) > maxDistance)
      {
        it = PASes.erase(it);
      }
      else
      {
        ++it;
      }
    }
}

void ContourSegmentNetwork::detectByHoughVoting( const ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects, bool visualizeHT, Mat &HTVisualization, HoughParams houghParams )
{
    //filterByDT( baseImage,  PASes );


    double t;
    //double t = (double)getTickCount();
    //filterPASes();
    //t = ((double)getTickCount() - t)/getTickFrequency();
    //static double houghFilter = 0;
    //houghFilter += t;
    //printf("houghFilter: %f\n", houghFilter);


    vector<Point2f> trainPoints;
    for( size_t i=0;i<PASes.size();i++ )
    {
        trainPoints.push_back( PASes[i].descriptor.intersectPoint );
        //printf("%f, %f\n", PASes[i].descriptor.intersectPoint.x, PASes[i].descriptor.intersectPoint.y);
    }


/*
    if( trainPoints.empty() )
    {
        printf("Start finding\n");
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Mat edges = calcEdges( baseImage, 100, 50);
        findContours( edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point() );
        for( size_t k=0;k<hierarchy.size();k++)
        {
            printf("[%d %d %d %d]\n", hierarchy[k][0],  hierarchy[k][1], hierarchy[k][2], hierarchy[k][3] );
        }
        printf("Found\n");


        int contourIdx=16;
        for( int contourIdx=0;contourIdx>=0;contourIdx=hierarchy[contourIdx][0] )
        {
            Mat img = Mat( baseImage.rows, baseImage.cols, CV_8UC1, Scalar(0));
            printf("contourIdx %d\n", contourIdx);
            //if( hierarchy[contourIdx][2] >= 0)
            //{
                //contourIdx = hierarchy[contourIdx][2];
            //}


        //for( size_t i=0;i<contours.size();i++ )
        {
            for( size_t j=0;j<contours[contourIdx].size();j++ )
                circle(img, contours[contourIdx][j], 0, Scalar(100) );
            imshow("img", img);
            waitKey(0);


            for( int k=hierarchy[contourIdx][2];k>=0;k=hierarchy[k][0] )
            {
                for( size_t j=0;j<contours[k].size();j++ )
                    circle(img, contours[k][j], 0, Scalar(150) );
                imshow("img", img);
                waitKey(0);

                for( int kk=hierarchy[k][2];kk>=0;kk=hierarchy[kk][0] )
                {
                    for( size_t j=0;j<contours[kk].size();j++ )
                        circle(img, contours[kk][j], 0, Scalar(255) );
                    imshow("img", img);
                    waitKey(0);

                }

            }

        }
        }
        printf("YES!!!!!!!\n");
    }
*/

    //double minVal;
    //double maxVal;
    //minMaxLoc(  Mat( trainPoints ), &minVal, &maxVal );
    //printf("%f     %f\n", minVal, maxVal);
    Rect trainBoundingRect = boundingRect( Mat( trainPoints ) );
    //printf("Ok!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    const double boundingRectScale = sqrt( 2. );
    trainBoundingRect.x -= trainBoundingRect.width * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.y -= trainBoundingRect.height * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.width *= boundingRectScale;
    trainBoundingRect.height *= boundingRectScale;

    //printf("Bounding rect: [%d, %d, %d, %d]\n", trainBoundingRect.x, trainBoundingRect.y, trainBoundingRect.width, trainBoundingRect.height  );

    const int nx = houghParams.nx;
    const int ny = houghParams.ny;
    const int nscale = houghParams.nscale;
    const double minLgScale = houghParams.minLgScale;
    const double maxLgScale = houghParams.maxLgScale;
    const int houghSpaceSize[] = {2*nx, 2*ny, nscale};
    MatND houghSpace (3, houghSpaceSize, CV_32F, Scalar(0) );

    float dissimilarityThreshold = houghParams.dissimilarityThreshold;


    Point2d maxTranslation = Point2d( testCSN.getCols(), testCSN.getRows() );
    double binWidth = maxTranslation.x / nx;
    double binHeight = maxTranslation.y / ny;
    double binScale = (maxLgScale - minLgScale) / nscale;


    //t = (double)getTickCount();
    for( size_t trainPASIdx=0;trainPASIdx<PASes.size();trainPASIdx++ )
    {
        for( size_t testPASIdx=0;testPASIdx<testCSN.PASes.size();testPASIdx++ )
        {
            PAS trainPAS = PASes[ trainPASIdx ];
            PAS testPAS = testCSN.PASes[ testPASIdx ];
            float distance = PAS::PASDescriptor::dissimilarity( trainPAS.descriptor, testPAS.descriptor );

            if( distance < dissimilarityThreshold )
            {
                assert( trainPAS.scale > 1e-6 /*just any epsilon*/);
                double lgScale = log10( testPAS.scale / trainPAS.scale );

                if( lgScale<minLgScale || lgScale>maxLgScale )
                    continue;

                int binS = floor( ( lgScale - minLgScale ) / binScale );
                assert( 0 <= binS && binS < nscale );

                Point2f translation = testPAS.location - trainPAS.location;
                //int binX = nx + floor( translation.x / binWidth );
                //int binY = ny + floor( translation.y / binHeight );
                //assert( 0 <= binX && binX < 2*nx );
                //assert( 0 <= binY && binY < 2*ny );


                int binX = nx + floor( translation.x / binWidth - 0.5 );
                int binY = ny + floor( translation.y / binHeight - 0.5 );
                if( binX == -1 )
                    binX = 0;
                if( binY == -1 )
                    binY = 0;
                assert( 0 <= binX && binX < 2*nx-1 );
                assert( 0 <= binY && binY < 2*ny-1 );
                float x = translation.x / binWidth  - 0.5 - floor( translation.x / binWidth  - 0.5 );
                float y = translation.y / binHeight - 0.5 - floor( translation.y / binHeight - 0.5 );


                float vote = (1 - distance / dissimilarityThreshold );
                //houghSpace.at<float>( binX, binY, binS) += (1-x)*(1-y)*vote;
                //houghSpace.at<float>( binX+1, binY, binS) += x*(1-y)*vote;
                //houghSpace.at<float>( binX, binY+1, binS) += (1-x)*y*vote;
                //houghSpace.at<float>( binX+1, binY+1, binS) += x*y*vote;

                //TODO: use edge length and strength
                float weight = 1;
                //weight = ( trainPAS.strength + testPAS.strength );
                weight = (trainPAS.descriptor.length1 + trainPAS.descriptor.length2 + testPAS.descriptor.length1 + testPAS.descriptor.length2) / 4;
                houghSpace.at<float>( binX, binY, binS) += weight*vote;
            }
        }
    }
    //t = ((double)getTickCount() - t)/getTickFrequency();
    //static double houghTrain = 0;
    //houghTrain += t;
    //printf("houghTrain: %f\n", houghTrain);

    double maxHough;
    minMaxLoc( houghSpace, 0, &maxHough );

    //float houghThreshold = ( PASes.size() / 3.) * 0.5;
    float houghThreshold = calcHoughThreshold( houghParams );
    //printf("MaxHough: %f;  HoughThreshold: %f\n", maxHough, houghThreshold );
    //maxHough = houghThreshold;

    //t = (double)getTickCount();
    for( int s=0;s<nscale;s++ )
    {
        //printf("Scale:  %d\n -------------------------------", s );
        for( int y=0;y<2*ny;y++ )
        {
            for( int x=0;x<2*nx;x++ )
            {
                //printf("%f ", houghSpace[x][y][s] );

                if( houghSpace.at<float>( x, y, s ) < houghThreshold )
                {
                    //houghSpace[x][y][s] = 0;
                }
                else
                {
                    //TODO: use scale
                    if( s != nscale/2 )
                        continue;

                    Rect rect = trainBoundingRect;
                    Point2f bin2 = Point2f( binWidth/2., binHeight/2.);
                    Point2f trans = Point2f( -maxTranslation.x + x*binWidth, -maxTranslation.y + y*binHeight ) + bin2;

                    //double lgScale = minLgScale + (s + 0.5)*binScale;
                    //double scale = pow10( lgScale );
                    //TODO: scale rectangle

                    rect.x += trans.x - bin2.x;
                    rect.y += trans.y - bin2.y;
                    rect.width += binWidth;
                    rect.height += binHeight;
                    boundingRects.push_back( std::pair<Rect, double> ( rect, houghSpace.at<float>( x, y, s ) ) );
                }
            }
            //printf("\n");
        }
    }
    //t = ((double)getTickCount() - t)/getTickFrequency();
    //static double houghResults = 0;
    //houghResults += t;
    //printf("houghResults: %f\n", houghResults);

    if( visualizeHT )
        HTVisualization = visualizeHoughTransform(testCSN, houghSpace, maxTranslation, houghParams, boundingRects);
}

Mat ContourSegmentNetwork::visualizeHoughTransform(const ContourSegmentNetwork &testCSN, const MatND &houghSpace, Point2d maxTranslation, const HoughParams &houghParams, const vector< std::pair<Rect, double> > &boundingRects) const
{
    //const Mat* img1 = &testImage;
    //const Mat* img2 = &baseImage;
    const Mat img1 = testCSN.drawAllSegments();
    const Mat img2 = drawAllSegments();


    double binWidth = maxTranslation.x / houghParams.nx;
    double binHeight = maxTranslation.y / houghParams.ny;

    float houghThreshold = calcHoughThreshold( houghParams );
    //TODO: use scale
    //for( int s=0;s<houghParams.nscale;s++ )
    int s = houghParams.nscale/2;
    {
        Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
        Mat drawImg(size, CV_MAKETYPE(img1.depth(), 3));
        drawImg.setTo(Scalar::all(0));
        Mat drawImg1 = drawImg(Rect(0, 0, img1.cols, img1.rows));
        Mat houghImg( img1.rows, img1.cols, CV_8UC3, Scalar(0) );
        Mat img1Color( img1.rows, img1.cols, CV_8UC3 );
        cvtColor( img1, img1Color, CV_GRAY2RGB );
        Mat drawImg2 = drawImg(Rect(img1.cols, 0, img2.cols, img2.rows));
        cvtColor( img2, drawImg2, CV_GRAY2RGB );
        Point2f center = PASes[0].location;
        circle( drawImg, center+Point2f(img1.cols, 0), 3, Scalar(0, 0, 255), CV_FILLED );


        vector<KeyPoint> keypoints = PASesToKeypoints();
        for( size_t kp = 0;kp<keypoints.size();kp++ )
        {
            circle( drawImg2, keypoints[kp].pt, 2, Scalar(0,255,0) );
        }

        //printf("Scale:  %d\n -------------------------------", s );

        for( int y=0;y<2*houghParams.ny;y++ )
        {
            for( int x=0;x<2*houghParams.nx;x++ )
            {
                int alpha = 255.* houghSpace.at<float>( x, y, s ) / houghThreshold;
                Point2f bin2 = Point2f( binWidth/2., binHeight/2.);
                Point2f trans = Point2f( -maxTranslation.x + x*binWidth, -maxTranslation.y + y*binHeight ) + bin2;
                Point2f lucorner = center + trans - bin2;
                Point2f rdcorner = center+trans+bin2;
                //if( lucorner.x < 0 || lucorner.y<0 || rdcorner.x>img1.cols || rdcorner.y>img1.rows)
                    //continue;
                //rectangle(drawImg, lucorner, rdcorner, color, CV_FILLED );
                rectangle(houghImg, lucorner, rdcorner, Scalar( alpha, 0, 0 ), CV_FILLED );
            }
        }

        addWeighted( houghImg, 0.5, img1Color, 0.5, 0, drawImg1 );

        for( int rectIdx=0;rectIdx<boundingRects.size();rectIdx++ )
        {
            Rect rect = boundingRects[rectIdx].first;
            rectangle( drawImg1, rect, Scalar(0,255,0) );
        }

        vector<KeyPoint> keypoints2 = testCSN.PASesToKeypoints();
        for( size_t kp = 0;kp<keypoints2.size();kp++ )
        {
            //printf("%f, %f\n", keypoints2[kp].pt.x, keypoints2[kp].pt.y );
            circle( drawImg1, keypoints2[kp].pt, 2, Scalar(0,0,255) );
        }



        //imshow("Hough Space", drawImg);

        //show scaled object image
        //
        //Mat smallImg2;
        //double lgScale = minLgScale + (s + 0.5)*binScale;
        //double scale = pow10( lgScale );
        //resize(img2, smallImg2, Size(), scale, scale);
        //imshow( "scaled testImage", smallImg2 );

        //if( s == nscale/2 )
        {
            return drawImg;
            //waitKey(0);
        }
    }
}

void printMat( string name, const Mat &mat)
{
    printf("%s:\n", name.c_str());
    for( int i=0;i<mat.rows;i++ )
        for( int j=0;j<mat.cols;j++ )
            printf("%d ", mat.at<int>(i, j) );
    printf("\n\n");
}

struct DisjointSet
{
    std::list<int> data;

    DisjointSet( int elem )
    {
        data.push_back( elem );
    }

    void Union( DisjointSet &disjointSet )
    {
        data.splice( data.end(), disjointSet.data );
    }

    bool empty()
    {
        return data.empty();
    }

    int size()
    {
        return data.size();
    }
};

void cliquePartition( const Mat &samples, double d, Mat &bestLabels, Mat *centers, Mat *sizes )
{
    //TODO: write bestLabels
    //bestLabels = Mat( 1, samples.rows, CV_32SC1 );

    PASDissimilarity<float> distance;

    Mat dists( samples.rows, samples.rows, CV_32FC1 );
    for( int i=0;i<samples.rows;i++ )
    {
        for( int j=0;j<samples.rows;j++ )
        {
            dists.at<float>( i, j ) = d - distance( samples.ptr<float>( i ), samples.ptr<float>( j ), 1 );
        }
    }

    int cliqueCount = samples.rows;

    //list<DisjointSet> cliques;
    vector<DisjointSet> cliques;
    for( int i=0;i<samples.rows;i++ )
        cliques.push_back( DisjointSet( i ) );


    double calcMergingCostsTime = 0;
    Mat mergingCosts( cliqueCount, cliqueCount, CV_32FC1, Scalar(0) );

    int clique1Idx = 0;
    double t = (double)getTickCount();
    for( vector<DisjointSet>::iterator clique1=cliques.begin(); clique1!=cliques.end(); clique1++, clique1Idx++ )
    {
        vector<DisjointSet>::iterator clique2 = clique1;
        clique2++;
        int clique2Idx = clique1Idx+1;
        for( ; clique2!=cliques.end(); clique2++, clique2Idx++ )
        {
            for( std::list<int>::iterator vertex1 = clique1->data.begin(); vertex1 != clique1->data.end(); vertex1++ )
            {
                for( std::list<int>::iterator vertex2 = clique2->data.begin(); vertex2 != clique2->data.end(); vertex2++ )
                {
                    mergingCosts.at<float>( clique1Idx, clique2Idx ) += dists.at<float>( *vertex1, *vertex2 );
                }
            }
            assert( clique2Idx > clique1Idx );
            assert( 0 <= clique1Idx && clique1Idx < cliqueCount );
            assert( 0 <= clique2Idx && clique2Idx < cliqueCount );
            mergingCosts.at<float>( clique2Idx, clique1Idx ) = mergingCosts.at<float>( clique1Idx, clique2Idx );
        }
    }
    t = ((double)getTickCount() - t)/getTickFrequency();
    calcMergingCostsTime += t;
    printf("calcMergingCostsTime = %f; ", t );


    bool isMerged = true;
    while( isMerged )
    {
        printf( "CliqueCount = %d\n", cliqueCount );
        //printMat("labels", bestLabels);

        isMerged = false;

        t = (double)getTickCount();

        //printMat("mergingCosts", mergingCosts );
        Mat bestMergingCliques( 1, cliqueCount, CV_32SC1 );

        for( int i=0;i<cliqueCount;i++ )
        {
            double maxVal;
            Point maxLoc;
            minMaxLoc( mergingCosts.row(i), 0, &maxVal, 0, &maxLoc );
            //printf("MaxLoc #%d: %d, %d\n", i, maxLoc.x, maxLoc.y );
            bestMergingCliques.at<int>( 0, i ) = maxLoc.x;
        }

        //printMat( "bestMergingCliques", bestMergingCliques );

        int mergingCount = 0;
        for( int i=0;i<cliqueCount;i++ )
        {
            int bestClique = bestMergingCliques.at<int>( 0, i );
            if( bestClique <= i )
                continue;
            if( i != bestMergingCliques.at<int>( 0, bestClique ) )
                continue;
            if( mergingCosts.at<int>( i, bestClique) <= 0 )
                continue;


            cliques[i].Union( cliques[ bestClique ] );

            mergingCosts.row( i ) += mergingCosts.row( bestClique );
            mergingCosts.col( i ) += mergingCosts.col( bestClique );
            bestMergingCliques.at<int>( 0, bestClique ) = -1;

            isMerged = true;
            mergingCount++;
        }
        t = ((double)getTickCount() - t)/getTickFrequency();
        printf("MergingTime= %f; ", t );
        t = (double)getTickCount();


        cliqueCount -= mergingCount;
        for( vector<DisjointSet>::iterator clique=cliques.begin(); clique!=cliques.end(); )
        {
            if( clique->empty() )
                clique = cliques.erase( clique );
            else
                clique++;
        }

        Mat newMergingCosts( cliqueCount, cliqueCount, CV_32FC1, Scalar(0) );
        int oldI = 0;
        for( int i=0; i<cliqueCount; i++, oldI++ )
        {
            while( bestMergingCliques.at<int>( 0, oldI ) == -1 )
            {
                oldI++;
            }

            int oldJ = 0;
            for( int j=0; j<cliqueCount; j++, oldJ++ )
            {
                while( bestMergingCliques.at<int>( 0, oldJ ) == -1 )
                {
                    oldJ++;
                }

                assert( 0 <= oldI && oldI < cliqueCount+mergingCount );
                assert( 0 <= oldJ && oldJ < cliqueCount+mergingCount );

                newMergingCosts.at<float>( i, j ) = mergingCosts.at<float>( oldI, oldJ );
            }
        }

        for( int i=0;i<cliqueCount;i++ )
        {
            newMergingCosts.at<float>( i, i ) = 0;
        }

        mergingCosts = newMergingCosts;


        assert( cliqueCount == cliques.size() );
        t = ((double)getTickCount() - t)/getTickFrequency();
        printf("ErasingTime= %f; \n", t );

    }

    centers->create( cliqueCount, samples.cols, CV_32FC1 );
    sizes->create( cliqueCount, 1, CV_32SC1 );
    int cliqueIdx = 0;

    printf("Dissimilarities in clusters:\n");
    for( vector<DisjointSet>::iterator clique=cliques.begin();clique!=cliques.end();clique++, cliqueIdx++ )
    {
        //float minDissimilarity = std::numeric_limits<float>::max();
        float maxWeight = std::numeric_limits<float>::min();
        float maxDissimilarity = std::numeric_limits<float>::min();

        //int minVertex = -1;
        int maxVertex = -1;
        for( std::list<int>::iterator vertex1 = clique->data.begin(); vertex1 != clique->data.end(); vertex1++ )
        {
            //float dissimilarity = 0;
            float weight = 0;

            for( std::list<int>::iterator vertex2 = clique->data.begin(); vertex2 != clique->data.end(); vertex2++ )
            {
                //dissimilarity += dists.at<float>( *vertex1, *vertex2 );
                weight += dists.at<float>( *vertex1, *vertex2 );

                if( d - dists.at<float>( *vertex1, *vertex2 ) > maxDissimilarity )
                    maxDissimilarity = d - dists.at<float>( *vertex1, *vertex2 );
            }
            //if( dissimilarity < minDissimilarity )
            if( weight > maxWeight )
            {
                maxWeight = weight;
                maxVertex = *vertex1;

                //minDissimilarity = dissimilarity;
                //minVertex = *vertex1;
            }
        }


        printf("%f\n", maxDissimilarity );
        Mat center = centers->row( cliqueIdx );
        //samples.row( minVertex ).copyTo( center );
        samples.row( maxVertex ).copyTo( center );
        sizes->at<int>( cliqueIdx, 0 ) = clique->size();
    }
}

/*
void cliquePartition( const Mat &samples, double d, Mat &bestLabels, Mat *centers )
{
    //TODO: write bestLabels
    //bestLabels = Mat( 1, samples.rows, CV_32SC1 );

    PASDissimilarity<float> distance;

    Mat dists( samples.rows, samples.rows, CV_32FC1 );
    for( int i=0;i<samples.rows;i++ )
    {
        for( int j=0;j<samples.rows;j++ )
        {
            dists.at<float>( i, j ) = d - distance( samples.ptr<float>( i ), samples.ptr<float>( j ), 1 );
        }
    }

    int cliqueCount = samples.rows;

    //list<DisjointSet> cliques;
    vector<DisjointSet> cliques;
    for( int i=0;i<samples.rows;i++ )
        cliques.push_back( DisjointSet( i ) );

    bool isMerged = true;


    double calcMergingCostsTime = 0;


    while( isMerged )
    {
        printf( "CliqueCount = %d\n", cliqueCount );
        //printMat("labels", bestLabels);

        isMerged = false;
        Mat mergingCosts( cliqueCount, cliqueCount, CV_32FC1, Scalar(0) );

        int clique1Idx = 0;
        double t = (double)getTickCount();
        for( vector<DisjointSet>::iterator clique1=cliques.begin(); clique1!=cliques.end(); clique1++, clique1Idx++ )
        {
            vector<DisjointSet>::iterator clique2 = clique1;
            clique2++;
            int clique2Idx = clique1Idx+1;
            for( ; clique2!=cliques.end(); clique2++, clique2Idx++ )
            {
                for( std::list<int>::iterator vertex1 = clique1->data.begin(); vertex1 != clique1->data.end(); vertex1++ )
                {
                    for( std::list<int>::iterator vertex2 = clique2->data.begin(); vertex2 != clique2->data.end(); vertex2++ )
                    {
                        mergingCosts.at<float>( clique1Idx, clique2Idx ) += dists.at<float>( *vertex1, *vertex2 );
                    }
                }
                assert( clique2Idx > clique1Idx );
                assert( 0 <= clique1Idx && clique1Idx < cliqueCount );
                assert( 0 <= clique2Idx && clique2Idx < cliqueCount );
                mergingCosts.at<float>( clique2Idx, clique1Idx ) = mergingCosts.at<float>( clique1Idx, clique2Idx );
            }
        }
        t = ((double)getTickCount() - t)/getTickFrequency();
        calcMergingCostsTime += t;
        printf("calcMergingCostsTime = %f; ", t );

        t = (double)getTickCount();

        //printMat("mergingCosts", mergingCosts );
        Mat bestMergingCliques( 1, cliqueCount, CV_32SC1 );

        for( int i=0;i<cliqueCount;i++ )
        {
            double maxVal;
            Point maxLoc;
            minMaxLoc( mergingCosts.row(i), 0, &maxVal, 0, &maxLoc );
            //printf("MaxLoc #%d: %d, %d\n", i, maxLoc.x, maxLoc.y );
            bestMergingCliques.at<int>( 0, i ) = maxLoc.x;
        }

        //printMat( "bestMergingCliques", bestMergingCliques );

        int mergingCount = 0;
        for( int i=0;i<cliqueCount;i++ )
        {
            int bestClique = bestMergingCliques.at<int>( 0, i );
            if( bestClique <= i )
                continue;
            if( i != bestMergingCliques.at<int>( 0, bestClique ) )
                continue;
            if( mergingCosts.at<int>( i, bestClique) <= 0 )
                continue;


            cliques[i].Union( cliques[ bestClique ] );

            isMerged = true;
            mergingCount++;
        }
        t = ((double)getTickCount() - t)/getTickFrequency();
        printf("MergingTime= %f; ", t );
        t = (double)getTickCount();


        cliqueCount -= mergingCount;
        for( vector<DisjointSet>::iterator clique=cliques.begin(); clique!=cliques.end(); )
        {
            if( clique->empty() )
                clique = cliques.erase( clique );
            else
                clique++;
        }
        assert( cliqueCount == cliques.size() );
        t = ((double)getTickCount() - t)/getTickFrequency();
        printf("ErasingTime= %f; \n", t );

    }

    centers->create( cliqueCount, samples.cols, CV_32FC1 );
    int cliqueIdx = 0;
    for( vector<DisjointSet>::iterator clique=cliques.begin();clique!=cliques.end();clique++, cliqueIdx++ )
    {
        float minDissimilarity = std::numeric_limits<float>::max();
        int minVertex = -1;
        for( std::list<int>::iterator vertex1 = clique->data.begin(); vertex1 != clique->data.end(); vertex1++ )
        {
            float dissimilarity = 0;

            for( std::list<int>::iterator vertex2 = clique->data.begin(); vertex2 != clique->data.end(); vertex2++ )
            {
                dissimilarity += dists.at<float>( *vertex1, *vertex2 );
            }
            if( dissimilarity < minDissimilarity )
            {
                minDissimilarity = dissimilarity;
                minVertex = *vertex1;
            }
        }

        Mat center = centers->row( cliqueIdx );
        samples.row( minVertex ).copyTo( center );
    }
}*/

/*
void cliquePartition( const Mat &samples, double d, Mat &bestLabels, Mat *centers )
{
    bestLabels = Mat( 1, samples.rows, CV_32SC1 );

    int* labels = bestLabels.ptr<int>();
    //distance is symmetric
    PASDissimilarity<float> distance;

    Mat dists( samples.rows, samples.rows, CV_32FC1 );
    for( int i=0;i<samples.rows;i++ )
    {
        for( int j=0;j<samples.rows;j++ )
        {
            dists.at<float>( i, j ) = d - distance( samples.ptr<float>( i ), samples.ptr<float>( j ), 1 );
        }
    }

    //printMat( "dists", dists );


    //for( int i=0;i<samples.rows;i++ )
    //{
        //for( int j=i+1;j<samples.rows;j++ )
        //{
            //dists.at<float>( i, j ) = distance( samples.row(i), samples.row(j) );
        //}
    //}

    int cliqueCount = samples.rows;



//    for( int i=0;i<samples.rows;i++ )
//    {
//        for( int j=i+1;j<samples.rows;j++ )
//        {
//            if( labels[i] == labels[j] )
//                continue;

//            int minClique = std::min( labels[i], labels[j] );
//            int maxClique = std::max( labels[i], labels[j] );
//            mergingCosts.at<float>( minClique, maxClique ) += dists.at<float>( i, j );
//        }
//    }

    for( int i=0;i<samples.rows;i++ )
        labels[i] = i;

    bool isMerged = true;
    while( isMerged )
    {
        printf("CliqueCount = %d\n", cliqueCount);
        printMat("labels", bestLabels);
        isMerged = false;
        Mat mergingCosts( cliqueCount, cliqueCount, CV_32FC1, Scalar(0) );
        for( int i=0;i<samples.rows;i++ )
        {
            for( int j=0;j<samples.rows;j++ )
            {
                assert( 0 <= labels[i] && labels[i] < cliqueCount );
                assert( 0 <= labels[j] && labels[j] < cliqueCount );
                if( labels[i] == labels[j] )
                    continue;

                mergingCosts.at<float>( labels[i], labels[j] ) += dists.at<float>( i, j );
            }
        }
        //printMat("mergingCosts", mergingCosts );
        Mat bestMergingCliques( 1, cliqueCount, CV_32SC1 );

        for( int i=0;i<cliqueCount;i++ )
        {
            double maxVal;
            Point maxLoc;
            minMaxLoc( mergingCosts.row(i), 0, &maxVal, 0, &maxLoc );
            printf("MaxLoc #%d: %d, %d\n", i, maxLoc.x, maxLoc.y );
            bestMergingCliques.at<int>( 0, i ) = maxLoc.x;
        }

        printMat( "bestMergingCliques", bestMergingCliques );

        int mergingCount = 0;
        for( int i=0;i<cliqueCount;i++ )
        {
            int bestClique = bestMergingCliques.at<int>( 0, i );
            if( bestClique <= i )
                continue;
            if( i != bestMergingCliques.at<int>( 0, bestClique ) )
                continue;
            if( mergingCosts.at<int>( i, bestClique) <= 0 )
                continue;

            for( int j=0;j<bestLabels.cols;j++ )
            {
                if( labels[j] > bestClique )
                    labels[j]--;
                else if( labels[j] == bestClique )
                    labels[j] = i;
            }
            isMerged = true;
            mergingCount++;
        }
        cliqueCount -= mergingCount;
    }


    Mat dissimilarities( 1, cliqueCount, CV_32FC1, Scalar( std::numeric_limits<float>::max() ) );
    Mat centerIndices( 1, cliqueCount, CV_32S );
    for( int i=0;i<samples.rows;i++ )
    {
        double dissimilarity = 0;
        for( int j=0;j<samples.rows;j++ )
        {
            if( labels[i] != labels[j] )
                continue;

            dissimilarity += dists.at<float>( i, j );
        }

        if( dissimilarity < dissimilarities.at<float>( 0, labels[i] ) )
        {
            dissimilarities.at<float>( 0, labels[i] ) = dissimilarity;
            centerIndices.at<int>( 0, labels[i] ) = i;
        }
    }

    centers->create( cliqueCount, samples.cols, CV_32FC1 );
    for( int cliqueIdx=0;cliqueIdx<cliqueCount;cliqueIdx++ )
    {
        Mat center = centers->row( cliqueIdx );
        samples.row( centerIndices.at<int>( 0, cliqueIdx ) ).copyTo( center );
    }
}*/

void visualizeCodebook( const Mat &codebook, const Mat &idf )
{
    for( int i=0;i<idf.cols;i++ )
    {
        printf("[%d]: %f\n ", i, idf.at<float>(0, i ));
    }
    printf("\n");


    Mat sortedIndices;
    sortIdx( idf, sortedIndices, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING );
    printf("Size: %d, %d\n ", sortedIndices.rows, sortedIndices.cols );


    float scale = 1;
    int width = 1024;
    int height = 800;
    Point center = Point( width/2, height/2 );
    for( int i=0;i<codebook.rows;i++ )
    {
        Mat img( height, width, CV_8UC3, Scalar( 0 ) );
        for( int k=0;k<7;k++ )
            printf("%f ", codebook.at<float>(sortedIndices.at<int>( 0, i ), k) );
        printf("\n");

        printf("IDF[%d]: %f\n ", sortedIndices.at<int>( 0, i ), idf.at<float>( 0, sortedIndices.at<int>( 0, i ) ) );
        PAS::PASDescriptor descriptor = codebook.ptr<float>( sortedIndices.at<int>( 0, i ) );
        descriptor.length1 *= scale;
        descriptor.length2 *= scale;
        descriptor.rVector *= scale;

        float x = descriptor.length1 * cos( descriptor.orientation1 );
        float y = descriptor.length1 * cos( descriptor.orientation1 );
        Point end1 = center + Point( x, y );
        line( img, center, end1, Scalar( 255, 0, 0 ) );
        Point2d midpoint1 = ( center + end1 ) * 0.5;
        Point2d rVector = descriptor.rVector;

        Point2d midpoint2 = midpoint1 + rVector;
        Point2d shift2 = descriptor.length2 * Point2d( cos( descriptor.orientation2 ), sin( descriptor.orientation2 ) ) * 0.5;
        Point2d back2 = midpoint2 + shift2;
        Point2d front2 = midpoint2 - shift2;
        line( img, front2, back2, Scalar( 0, 0, 255 ) );

        imshow( "PAS", img);
        waitKey(0);
    }

}

void sortCodebook( Mat &codebook, Mat &idf )
{
    Mat sortedIndices;
    sortIdx( idf, sortedIndices, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING );
    Mat sortedIdf;
    sort( idf, sortedIdf, CV_SORT_EVERY_ROW + CV_SORT_DESCENDING );

    Mat sortedCodebook( codebook.rows, codebook.cols, codebook.type() );
    for( int i=0;i<codebook.rows;i++ )
    {
        Mat row = sortedCodebook.row( i );
        codebook.row( sortedIndices.at<int>( 0, i ) ).copyTo( row );
    }

    codebook = sortedCodebook;
    idf = sortedIdf;
}

void constructCodebook( const vector<ContourSegmentNetwork*> &csns, Mat &codebook, double dissimilarityThreshold )
{
    int size = 0;
    for( size_t i=0;i<csns.size();i++ )
    {
        size += csns[i]->PASes.size();
    }
    printf("Samples size = %d", size );


    Mat samples( size, PAS::PASDescriptor::descriptorLength, CV_32FC1);
    int rowIdx = 0;
    for( size_t i=0;i<csns.size();i++ )
    {
        for( size_t j=0;j<csns[i]->PASes.size();j++ )
        {
            Mat sample = samples.row( rowIdx );
            Mat pas = csns[i]->PASes[j];
            pas.copyTo( sample );
            rowIdx++;
        }
    }

    Mat labels;
    Mat sizes;
    cliquePartition( samples, dissimilarityThreshold, labels, &codebook, &sizes );

    printf("Codebook size: %d\n", codebook.rows);
    for( int i=0;i<codebook.rows;i++ )
    {
        for( int k=0;k<7;k++ )
            printf("%f ", codebook.at<float>(i,k) );
        printf("\n");
    }

    //visualizeCodebook( codebook, sizes );
}

void calcIDF( const vector<ContourSegmentNetwork*> &csns, const Mat &codebook, Mat &idf, double dissimilarityThreshold )
{
    idf.create( 1, codebook.rows, CV_32FC1 );
    for( int bin=0;bin<=codebook.rows;bin++ )
    {
        PAS::PASDescriptor codeword = codebook.ptr<float>( bin );

        int ni = 0;
        for( size_t i=0;i<csns.size();i++ )
        {
            for( size_t j=0;j<csns[i]->PASes.size();j++ )
            {
                double distance = PAS::PASDescriptor::dissimilarity( codeword, csns[i]->PASes[j].descriptor );
                if( distance < dissimilarityThreshold )
                {
                    ni++;
                    break;
                }
            }
        }

        idf.at<float>( 0, bin ) = log( csns.size() / ( (float) ni ) );
    }
}

void computeHistogram( const vector<PAS> &pases, Rect tile, const Mat &codebook, MatND &histogram, double dissimilarityThreshold )
{
    const int ndims = 1;
    const int sizes[] = { codebook.rows };
    histogram = MatND( ndims, sizes, CV_32FC1, Scalar(0) );
    const double pasWeight = 1.;

    for( size_t i=0;i<pases.size();i++ )
    {
        PAS pas = pases[ i ];

        if( !tile.contains( pas.descriptor.intersectPoint ) )
            continue;

        double sum = 0.;
        vector<int> bins;
        double epsilon = 1e-4;
        for( int j=0;j<codebook.rows;j++ )
        {
            PAS::PASDescriptor codeword = codebook.ptr<float>( j );
            double distance = PAS::PASDescriptor::dissimilarity( codeword, pas.descriptor );
            if( distance < dissimilarityThreshold )
            {
                if( distance < epsilon )
                    distance = epsilon;

                sum += 1. / distance;
                bins.push_back( j );
                //histogram.at<float>( j ) += (1 - distance / dissimilarityThreshold );
            }
        }

        for( size_t j=0;j<bins.size();j++ )
        {
            PAS::PASDescriptor codeword = codebook.ptr<float>( bins[j] );
            double distance = PAS::PASDescriptor::dissimilarity( codeword, pas.descriptor );
            if( distance < epsilon )
                distance = epsilon;
            assert( distance < dissimilarityThreshold );

            histogram.at<float>( bins[j] ) += ( pasWeight / sum ) / distance;
        }
    }
}

void computeHistogram( const PAS &pas, const Mat &codebook, MatND &histogram, double dissimilarityThreshold )
{
    vector<PAS> fakePASes;
    fakePASes.push_back( pas );
    Rect fakeTile = Rect( 0, 0, std::numeric_limits<int>::max(), std::numeric_limits<int>::max() );
    computeHistogram( fakePASes, fakeTile, codebook, histogram, dissimilarityThreshold);
}

void computeHistogram( const MatND &IH, Rect tile, int codebookSize, MatND &histogram, double dissimilarityThreshold )
{
    assert( histogram.dims == 1 );
    assert( histogram.size[0] == codebookSize );
    assert( histogram.type() == CV_32FC1 );

    float *histLU = (float *) IH.ptr( tile.x, tile.y );
    float *histRU = (float *) IH.ptr( tile.x+tile.width, tile.y );
    float *histLD = (float *) IH.ptr( tile.x, tile.y+tile.height );
    float *histRD = (float *) IH.ptr( tile.x + tile.width, tile.y + tile.height );

    float *hist = (float *) histogram.ptr( 0 );

    for( int bin=0;bin<codebookSize;bin++ )
    {
        hist[bin] = histRD[bin] - histRU[bin] - histLD[bin] + histLU[bin];
    }

//    for( int bin=0;bin<codebookSize;bin++ )
//    {
//        histogram.at<float>( bin ) = IH.at<float>( tile.x + tile.width, tile.y + tile.height, bin ) - IH.at<float>( tile.x+tile.width, tile.y, bin ) - IH.at<float>( tile.x, tile.y+tile.height, bin ) + IH.at<float>( tile.x, tile.y, bin );
        //histogram.at<float>( bin ) = IH.at<double>( tile.x + tile.width, tile.y + tile.height, bin ) - IH.at<double>( tile.x+tile.width, tile.y, bin ) - IH.at<double>( tile.x, tile.y+tile.height, bin ) + IH.at<double>( tile.x, tile.y, bin );
//    }
}



double compareHistograms( const MatND &trainHist, const MatND &testHist, const Mat &idf )
{
    assert( trainHist.size[0] == testHist.size[0] );
/*
// ********** Naive Bayes *******
    double trainHistSum = sum( trainHist )[0];
    double result = 0;
    for( int bin=0;bin<trainHist.size[0];bin++ )
    {
        double pvt = ( 1 + trainHist.at<float>( bin ) ) / ( trainHist.size[0] + trainHistSum );
        result += testHist.at<float>( bin ) * log( pvt );
    }
    double testHistSum = sum( testHist )[0];
    double epsilon = 1e-4;
    if( testHistSum < epsilon )
        return 5;
    result /= testHistSum;


    return -result;
*/

    double trainHistNormalizer = 0;
    double testHistNormalizer = 0;
    double result = 0;
/*    for( int i=0;i<trainHist.size[0];i++ )
    {
        //double diff = idf.at<float>( i ) * ( trainHist.at<float>( i ) - testHist.at<float>( i ) );
        double diff = ( trainHist.at<float>( i ) - testHist.at<float>( i ) );
        result += diff*diff;
        trainHistNormalizer += trainHist.at<float>( i ) * trainHist.at<float>( i );
        //double idfSq = idf.at<float>( i ) * idf.at<float>( i );
        //trainHistNormalizer += idfSq * trainHist.at<float>( i ) * trainHist.at<float>( i );
    }
    double epsilon = 1e-5;
    if( trainHistNormalizer < epsilon )
        return 0;
    result = sqrt( result ) / sqrt( trainHistNormalizer );
    return result;
*/


//L2

    for( int i=0;i<trainHist.size[0];i++ )
    {
        double diff = idf.at<float>( i ) * ( trainHist.at<float>( i ) - testHist.at<float>( i ) );
        result += diff*diff;
        //result += fabs( diff );

        double idfSq = idf.at<float>( i ) * idf.at<float>( i );
        trainHistNormalizer += idfSq * trainHist.at<float>( i ) * trainHist.at<float>( i );
        testHistNormalizer += idfSq * testHist.at<float>( i ) * testHist.at<float>( i );
        //testHistNormalizer += fabs(idf.at<float>( i )) * testHist.at<float>( i );
        //trainHistNormalizer += fabs(idf.at<float>( i )) * trainHist.at<float>( i );
    }

    //double epsilon = 1e-5;
    //if( trainHistNormalizer < epsilon )
        //return 5;
    //result = sqrt( result );
    result /= 0.001 + sqrt( trainHistNormalizer );
    //result /= ( exp( trainHistNormalizer ) );
    //result /= ( exp( testHistNormalizer ) );
    result /= 0.001 + sqrt( testHistNormalizer );

    //result /= 0.1 + trainHistNormalizer;
    //result /= 0.1 + testHistNormalizer;

    return -result;


/* * Cosine similarity
    for( int i=0;i<trainHist.size[0];i++ )
    {
        //double tf = trainHist.at<float>( i );
        //double weight = tf * idf.at<float>( i );

        //double diff = trainHist.at<float>( i ) - testHist.at<float>( i );

        double idfSq = idf.at<float>( 0, i ) * idf.at<float>( 0, i );
        result += idfSq * trainHist.at<float>( i ) * testHist.at<float>( i );

        trainHistNormalizer += idfSq * trainHist.at<float>( i ) * trainHist.at<float>( i );
        testHistNormalizer += idfSq * testHist.at<float>( i ) * testHist.at<float>( i );
    }
    double epsilon = 1e-5;
    if( trainHistNormalizer < epsilon || testHistNormalizer < epsilon )
        return 0;
    //if( trainHistNormalizer < epsilon && testHistNormalizer < epsilon )
        //return -1;
    //if( trainHistNormalizer < epsilon )
        //return -1. / sqrt( testHistNormalizer );
    //if( testHistNormalizer < epsilon )
        //return -1. / sqrt( trainHistNormalizer );

    result /= sqrt( trainHistNormalizer );
    result /= sqrt( testHistNormalizer );
    return result;
*/



//* * Cosine similarity optimized
//* The best!
    /*
    float *train = (float *) trainHist.data;
    float *test = (float *) testHist.data;
    float *weights = (float *) idf.data;
    for( int i=0;i<trainHist.size[0];i++ )
    {
        //double trainWeight = weights[i] * train[i];
        double testWeight = weights[i] * test[i];
        //result += trainWeight * testWeight;
        result += train[i] * testWeight;

        //trainHistNormalizer += trainWeight * trainWeight;
        testHistNormalizer += testWeight * testWeight;
    }
    double epsilon = 1e-5;
    //if( trainHistNormalizer < epsilon || testHistNormalizer < epsilon )
    if( testHistNormalizer < epsilon )
        return 0;

    //result /= sqrt( trainHistNormalizer );
    result /= sqrt( testHistNormalizer );
    return -result;
*/
//TODO: process empty tiles
    /*
    float *train = (float *) trainHist.data;
    float *test = (float *) testHist.data;
    float *weights = (float *) idf.data;
    for( int i=0;i<trainHist.size[0];i++ )
    {
        double trainWeight = weights[i] * train[i];
        double testWeight = weights[i] * test[i];
        result += trainWeight * testWeight;
        //result += train[i] * testWeight;

        trainHistNormalizer += trainWeight * trainWeight;
        testHistNormalizer += testWeight * testWeight;
    }
    double epsilon = 1e-5;
    if( trainHistNormalizer < epsilon && testHistNormalizer < epsilon )
        return -0.5;
    if( testHistNormalizer < epsilon )
        return std::max( -1. / trainHistNormalizer, -0.5 );
    if( trainHistNormalizer < epsilon )
        return std::max( -1. / testHistNormalizer, -0.5 );


    result /= sqrt( trainHistNormalizer );
    result /= sqrt( testHistNormalizer );
    return -result;
*/


/* ************* Best subset

    float *train = (float *) trainHist.data;
    float *test = (float *) testHist.data;
    float *weights = (float *) idf.data;
    double epsilon = 1e-4;
    double trainNoise = 0;
    double testNoise = 0;
    for( int i=0;i<trainHist.size[0];i++ )
    {
        //double trainWeight = weights[i] * train[i];
        double testWeight = weights[i] * test[i];
        if( fabs( testWeight ) < epsilon || fabs( train[i] ) < epsilon )
        {
            trainNoise += train[i] * train[i];
            testNoise += testWeight * testWeight;
            continue;
        }

        //result += trainWeight * testWeight;
        result += train[i] * testWeight;

        trainHistNormalizer += train[i] * train[i];
        testHistNormalizer += testWeight * testWeight;

    }
    if( trainHistNormalizer < epsilon || testHistNormalizer < epsilon )
        return 0;
    if( testHistNormalizer < epsilon )
        return 0;

    result /= sqrt( trainHistNormalizer );
    result /= sqrt( testHistNormalizer );

    result /= (1 + sqrt( trainNoise * testNoise ) );
    return -result;
*/














    //return compareHist( hist1, hist2, CV_COMP_CHISQR );

/*
    double result = 0;
    double trainHistNormalizer = 0;
    double testHistNormalizer = 0;
    for( int i=0;i<trainHist.size[0];i++ )
    {
        trainHistNormalizer += trainHist.at<float>( i );
        testHistNormalizer += testHist.at<float>( i );
    }
    for( int i=0;i<trainHist.size[0];i++ )
    {
        result += log( trainHist.at<float>( i ) / trainHistNormalizer ) * testHist.at<float>( i ) / testHistNormalizer;
    }
    return -result;
*/
}

/*
double compareTiles( const vector<PAS> &testPASes, const vector<PAS> &PASes, const Rect &testTile, const Rect &trainTile, const Mat &codebook )
{
    //return -1;
    //MatND testHist;
    //computeHistogram( testPASes, testTile, codebook, testHist );
    //MatND trainHist;
    //computeHistogram( PASes, trainTile, codebook, trainHist );
    //return compareHistograms( trainHist, testHist );


    //*
     //* ****************** Compare tiles by my ad-hoc function
     //*

    float dissimilarityThreshold = 3;
    double result = 0;
    for( size_t testPASIdx=0;testPASIdx<testPASes.size();testPASIdx++ )
    {
        PAS testPAS = testPASes[ testPASIdx ];
        if( !testTile.contains( testPAS.descriptor.intersectPoint ) )
            continue;

        for( size_t trainPASIdx=0;trainPASIdx<PASes.size();trainPASIdx++ )
        {
            PAS trainPAS = PASes[ trainPASIdx ];
            if( !trainTile.contains( trainPAS.descriptor.intersectPoint ) )
                continue;


            double distance = PAS::PASDescriptor::dissimilarity( trainPAS.descriptor, testPAS.descriptor );

            if( distance < dissimilarityThreshold )
            {
                result += (1 - distance / dissimilarityThreshold );
            }
        }
    }
    return result;
//
}*/

MatND computeIntegralHistogram( const Mat &codebook, const ContourSegmentNetwork &testCSN, double dissimilarityThreshold )
{
	bool verbose = false;
    double t = (double)getTickCount();
    const int ndims = 3;
    const int sizes[] = { testCSN.getCols() + 1, testCSN.getRows() + 1, codebook.rows };


    //integralHistogram.at<float>( i, j, *) is histogram of region: 0 <= x < i, 0 <= y < j
    MatND integralHistogram( ndims, sizes, CV_32FC1, Scalar(0) );
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("t1: %f; ", t);

    t = (double)getTickCount();
    for( int pasIdx=0;pasIdx<testCSN.PASes.size();pasIdx++ )
    {
        MatND Qxy;
        computeHistogram( testCSN.PASes[ pasIdx ], codebook, Qxy, dissimilarityThreshold );
        assert( Qxy.dims == 1 && Qxy.size[0] == codebook.rows );
        Point pt = testCSN.PASes[ pasIdx ].descriptor.intersectPoint;
        if ( !( 0 <= pt.x +1 && pt.x + 1 < sizes[0] ) )
        {
            printf("pt: %d, %f\n", pt.x, testCSN.PASes[ pasIdx ].descriptor.intersectPoint.x);
            assert( false );
        }
        if ( !( 0 <= pt.y +1 && pt.y + 1 < sizes[1] ) )
        {
            printf("pt: %d, %f\n", pt.y, testCSN.PASes[ pasIdx ].descriptor.intersectPoint.y);
            assert( false );
        }

        for( int bin=0;bin<codebook.rows;bin++ )
        {
            integralHistogram.at<float>( pt.x+1, pt.y+1, bin ) += Qxy.at<float>( bin );
        }
    }
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("Init: %f; ", t);

    t = (double)getTickCount();
    for( int x=1;x<=testCSN.getCols();x++ )
    {
        for( int y=1;y<=testCSN.getRows();y++ )
        {
            for( int bin=0;bin<codebook.rows;bin++ )
            {
                integralHistogram.at<float>( x, y, bin ) += integralHistogram.at<float>( x-1, y, bin ) + integralHistogram.at<float>( x, y-1, bin ) - integralHistogram.at<float>( x-1, y-1, bin );
            }
        }
    }
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("Propagation: %f;\n", t);

    return integralHistogram;
}

void printHist( const MatND &hist, const Mat &idf, bool addWeight )
{
    assert( hist.dims == 1);
    assert( hist.type() == CV_32FC1);

    if( !addWeight )
    {
        for( int i=0;i<hist.size[0];i++ )
        {
            if( hist.at<float>( i ) <= 0 )
                printf("%f ", 0.);
            else
                printf("%f ", hist.at<float>( i ) );
        }
    }
    else
    {
        double histNormalizer = 0;
        float *weights = (float *) idf.data;
        float *train = (float *) hist.data;
        for( int i=0;i<hist.size[0];i++ )
        {
            double trainWeight = weights[i] * train[i];
            histNormalizer += trainWeight * trainWeight;
        }
        double epsilon = 1e-4;
        histNormalizer = sqrt( histNormalizer );
        printf("HistNormalizer: %f\n", histNormalizer);
        if( histNormalizer < epsilon )
            histNormalizer = 1;
        for( int i=0;i<hist.size[0];i++ )
        {
            if( ( weights[i] * train[i] ) / histNormalizer <= 0 )
                printf("%f ", 0. );
            else
                printf("%f ", ( weights[i] * train[i] ) / histNormalizer );
        }
    }

    printf("\n");
}

double getSum( const PAS &pas, Rect tile, const Mat &codebook, float dissimilarityThreshold )
{
    if( !tile.contains( pas.descriptor.intersectPoint) )
        return 0;
    double sum = 0.;
    double epsilon = 1e-4;
    for( int j=0;j<codebook.rows;j++ )
    {
        PAS::PASDescriptor codeword = codebook.ptr<float>( j );
        double distance = PAS::PASDescriptor::dissimilarity( codeword, pas.descriptor );
        if( distance < dissimilarityThreshold )
        {
            if( distance < epsilon )
                distance = epsilon;

            sum += 1. / distance;
        }
    }
    return sum;
}



double getMaxsum( const vector<PAS> &PASes, Rect tile, const Mat &codebook, float dissimilarityThreshold )
{
    double maxsum = 0;
    for( size_t kp = 0;kp<PASes.size();kp++ )
    {
        double sum = getSum( PASes[kp], tile, codebook, dissimilarityThreshold );
        if( sum > maxsum )
            maxsum = sum;
    }
    return maxsum;
}

void ContourSegmentNetwork::detectBySlidingWindow( const Mat &codebook, double dissimilarityThreshold, const MatND &integralHistogram, const ContourSegmentNetwork &testCSN, const Mat &idf, vector< std::pair<Rect, double> > &boundingRects, bool visualize, Mat &visualization ) const
{
	bool verbose = false;

    double t = (double)getTickCount();
    vector<Point2f> trainPoints;
    for( size_t i=0;i<PASes.size();i++ )
    {
        trainPoints.push_back( PASes[i].descriptor.intersectPoint );
    }
    Rect trainBoundingRect = boundingRect( Mat( trainPoints ) );

    const double boundingRectScale = sqrt( 1.2 );
    trainBoundingRect.x -= trainBoundingRect.width * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.y -= trainBoundingRect.height * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.width *= boundingRectScale;
    trainBoundingRect.height *= boundingRectScale;


    Rect imageRect = trainBoundingRect;

    const int stepX = 10;
    const int stepY = 10;
    //const int tilesNumber = 4;
    const int tilesNumber = 8;


    const int tilesX = round( sqrt( tilesNumber * ((float) imageRect.width) / imageRect.height ) );
    const int tilesY = round( sqrt( tilesNumber * ((float) imageRect.height) / imageRect.width ) );
    //const int tilesX = 1;
    //const int tilesY = 1;
    const int tileWidth = (imageRect.width) / tilesX;
    const int tileHeight = (imageRect.height) / tilesY;
    const int residualTileWidth = tileWidth + ( imageRect.width % tilesX );
    const int residualTileHeight = tileHeight + ( imageRect.height % tilesY );
    Rect testTile;
    Rect trainTile;

    const int ndims = 1;
    const int sizes[] = { codebook.rows };

    MatND **trainHists = new MatND*[ tilesX ];
    for( int i=0;i<tilesX;i++ )
    {
        trainHists[i] = new MatND[ tilesY ];
        for( int j=0;j<tilesY;j++ )
        {
            trainTile.x = trainBoundingRect.x + i*tileWidth;
            trainTile.y = trainBoundingRect.y + j*tileHeight;
            trainTile.width = ( i == tilesX-1 ) ? residualTileWidth : tileWidth;
            trainTile.height = ( j == tilesY-1 ) ? residualTileHeight : tileHeight;

            MatND trainHist  = MatND( ndims, sizes, CV_32FC1 );
            computeHistogram( PASes, trainTile, codebook, trainHist, dissimilarityThreshold );
            //double normalizer = norm( trainHist );


            //normalize histogram
            double trainHistNormalizer = 0;
            float *weights = (float *) idf.data;
            float *train = (float *) trainHist.data;
            for( int i=0;i<trainHist.size[0];i++ )
            {
                double trainWeight = weights[i] * train[i];
                trainHistNormalizer += trainWeight * trainWeight;
            }
            double epsilon = 1e-4;
            trainHistNormalizer = sqrt( trainHistNormalizer );
/*
            if( trainHistNormalizer >= epsilon )
            {
                for( int i=0;i<trainHist.size[0];i++ )
                {
                    train[i] = ( weights[i] * train[i] ) / trainHistNormalizer;
                }
            }
*/
            if( verbose )
            	printf( "TrainHist[%d][%d]: normalizer = %f\n", i, j, trainHistNormalizer );

            trainHists[i][j] = trainHist;
            //printHist( trainHists[i][j], idf, false );

            //printf("{%d, %d}: ", j, i);
            for( int i=0;i<codebook.rows;i++ )
            {
                //printf("%f ", trainHist.at<float>( i ));
            }
            //printf("\n");
        }
    }



    double maxConfidence = -std::numeric_limits<double>::max();
    //double minError = std::numeric_limits<double>::max();
    Rect object;
    double computingTime = 0;
    double comparingTime = 0;
    t = ((double)getTickCount() - t)/getTickFrequency();
    double beforeTime = t;

    MatND testHist = MatND( ndims, sizes, CV_32FC1 );
    for( imageRect.y = 0; imageRect.y + imageRect.height < testCSN.getRows(); imageRect.y += stepY )
    {
        //for( int startX = 0; startX < imageRect.width; startX += stepX )
            //for( imageRect.x = startX; imageRect.x + imageRect.width < testCSN.getCols(); imageRect.x += imageRect.width )
        for( imageRect.x = 0; imageRect.x + imageRect.width < testCSN.getCols(); imageRect.x += stepX )
        {
            //double error = 0;
            double confidence = 0;


            for( int i=0;i<tilesX;i++ )
            {
                for( int j=0;j<tilesY;j++ )
                {
                    testTile.x = imageRect.x + i*tileWidth;
                    testTile.y = imageRect.y + j*tileHeight;
                    testTile.width = ( i == tilesX-1 ) ? residualTileWidth : tileWidth;
                    testTile.height = ( j == tilesY-1 ) ? residualTileHeight : tileHeight;

                    //confidence += compareTiles( testCSN.PASes, PASes, testTile, trainTile, codebook );
                    double t = (double)getTickCount();
                    //MatND oldHist;
                    //computeHistogram( testCSN.PASes, testTile, codebook, oldHist, dissimilarityThreshold );
                    computeHistogram( integralHistogram, testTile, codebook.rows, testHist, dissimilarityThreshold );
                    t = ((double)getTickCount() - t)/getTickFrequency();
                    computingTime += t;

                    t = (double)getTickCount();
                    //error += compareHistograms( trainHists[i][j], testHist, idf );
                    confidence += compareHistograms( trainHists[i][j], testHist, idf );


                    //double tmp = compareHistograms( trainHists[i][j], testHist, idf );
                    //error += -tmp*tmp;
                    t = ((double)getTickCount() - t)/getTickFrequency();

                    comparingTime += t;

                    //error -= compareTiles( testCSN.PASes, PASes, testTile, trainTile, codebook );

                }
            }
            //error /= ( tilesX * tilesY );
            confidence /= ( tilesX * tilesY );

            //error = - sqrt( -error );

            //assert( error < 1e-4 );
            //assert( -1 - 1e-4 < error );
            //assert( error <= 0.1 );
            //assert( -1 <= error );

            //printf("%f\n ", error);
            if( confidence > maxConfidence )
            {
                maxConfidence = confidence;
                object = imageRect;
            }
        }
    }
    if( verbose )
    	printf("Before: %f; Computing: %f; Comparing: %f\n", beforeTime, computingTime, comparingTime );

    boundingRects.push_back( std::pair<Rect, double>( object, maxConfidence ) );


    for( int i=0;i<tilesX;i++ )
    {
        for( int j=0;j<tilesY;j++ )
        {
            testTile.x = object.x + i*tileWidth;
            testTile.y = object.y + j*tileHeight;
            testTile.width = ( i == tilesX-1 ) ? residualTileWidth : tileWidth;
            testTile.height = ( j == tilesY-1 ) ? residualTileHeight : tileHeight;

            //confidence += compareTiles( testCSN.PASes, PASes, testTile, trainTile, codebook );
            MatND testHist = MatND( ndims, sizes, CV_32FC1 );
            computeHistogram( integralHistogram, testTile, codebook.rows, testHist, dissimilarityThreshold );

            if( verbose )
            	printf("Error[%d][%d]: %f\n", i, j, compareHistograms( trainHists[i][j], testHist, idf ) );
            //printHist( testHist, idf, false );
            //printHist( testHist, idf, true );
            //printf("%f ", compareHistograms( trainHists[i][j], testHist, idf ) );
        }
    }
    //printf("\n");



    if( visualize )
    {
        const Mat img1 = testCSN.drawAllSegments();
        const Mat img2 = drawAllSegments();


        Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
        Mat drawImg(size, CV_MAKETYPE(img1.depth(), 3));
        drawImg.setTo(Scalar::all(0));
        Mat drawImg1 = drawImg(Rect(0, 0, img1.cols, img1.rows));
        cvtColor( img1, drawImg1, CV_GRAY2RGB );
        Mat drawImg2 = drawImg(Rect(img1.cols, 0, img2.cols, img2.rows));
        cvtColor( img2, drawImg2, CV_GRAY2RGB );

        vector<KeyPoint> keypoints = PASesToKeypoints();
        assert( keypoints.size() == PASes.size() );
        vector<KeyPoint> keypoints2 = testCSN.PASesToKeypoints();
        assert( keypoints2.size() == testCSN.PASes.size() );

        for( int i=0;i<tilesX;i++ )
        {
            for( int j=0;j<tilesY;j++ )
            {
                trainTile.x = trainBoundingRect.x + i*tileWidth;
                trainTile.y = trainBoundingRect.y + j*tileHeight;
                trainTile.width = ( i == tilesX-1 ) ? residualTileWidth : tileWidth;
                trainTile.height = ( j == tilesY-1 ) ? residualTileHeight : tileHeight;
                rectangle( drawImg2, trainTile, Scalar(0,255,0) );

                Rect imageRect = boundingRects[0].first;
                testTile.x = imageRect.x + i*tileWidth;
                testTile.y = imageRect.y + j*tileHeight;
                testTile.width = ( i == tilesX-1 ) ? residualTileWidth : tileWidth;
                testTile.height = ( j == tilesY-1 ) ? residualTileHeight : tileHeight;
                rectangle( drawImg1, testTile, Scalar(0,255,0) );

                double sum, maxsum;
                //maxsum = getMaxsum( PASes, trainTile, codebook, dissimilarityThreshold );
                for( size_t kp = 0;kp<keypoints.size();kp++ )
                {
                    if( !trainTile.contains( keypoints[kp].pt ) )
                        continue;
                    //sum = getSum( PASes[kp], trainTile, codebook, dissimilarityThreshold );
                    //circle( drawImg2, keypoints[kp].pt, 2, Scalar( 0, 0, 255 * (sum / maxsum )) );
                    circle( drawImg2, keypoints[kp].pt, 2, Scalar( 0, 0, 255 ) );
                }

                //maxsum = getMaxsum( testCSN.PASes, testTile, codebook, dissimilarityThreshold );
                for( size_t kp = 0;kp<keypoints2.size();kp++ )
                {
                    if( !testTile.contains( keypoints2[kp].pt ) )
                        continue;

                    //sum = getSum( testCSN.PASes[kp], testTile, codebook, dissimilarityThreshold );
                    //circle( drawImg1, keypoints2[kp].pt, 2, Scalar(0,0,255 * (sum/maxsum)) );
                    circle( drawImg1, keypoints2[kp].pt, 2, Scalar(0,0,255) );
                }
            }
        }


        for( int rectIdx=0;rectIdx<boundingRects.size();rectIdx++ )
        {
            Rect rect = boundingRects[rectIdx].first;
            rectangle( drawImg1, rect, Scalar(0,255,0) );
        }

        visualization = drawImg;



        //return drawImg;
    }

    for( int i=0;i<tilesX;i++ )
    {
        delete []trainHists[i];
    }
    delete []trainHists;

}

Mat ContourSegmentNetwork::calcObjectEdges( const Mat &image, double threshold1, double threshold2 ) const
{
    Mat circleMask(image.size(), CV_8UC1);
    int size=1;

    Mat edges = calcEdges( image );


    assert( edges.type() == CV_8UC1 );
    assert( image.type() == CV_8UC1 );

    for( int i=0;i<image.rows;i++ )
    {
        for( int j=0;j<image.cols;j++ )
        {
            if( image.at<unsigned char>( i, j ) == 0 )
            {
                circle(edges, Point( j, i ), 2*size+1, CV_RGB(0, 0, 0), -1);
            }
        }
    }

    Mat result = filterEdgesByLength( edges );

/*

    for( int i=0;i<edges.rows;i++ )
    {
        printf("%d\n",i);
        for( int j=0;j<edges.cols;j++ )
        {
            if( edges.at<unsigned char>( i, j ) == 0 )
                continue;

            circleMask.setTo(Scalar::all(0));
            circle(circleMask, Point( j, i ), 2*size+1, CV_RGB(255, 255, 255), -1);
            if( countNonZero( baseImage==0 & circleMask ) )
            {
                edges.at<unsigned char>( i, j ) = 0;
            }
        }
    }
    */
    //TODO: remove
    //imwrite("test.png", edges);
    //exit(-1);
    return result;
}

Rect ContourSegmentNetwork::calcObjectBoundingBox( const Mat &edges )
{
    int minx = std::numeric_limits<int>::max();
    int miny = std::numeric_limits<int>::max();
    int maxx = -std::numeric_limits<int>::max();
    int maxy = -std::numeric_limits<int>::max();
    const int margin = 1;
    //for( int y=margin;y<edges.rows-margin;y++ )
    for( int y=0;y<edges.rows;y++ )
    {
        for( int x=0;x<edges.cols;x++ )
        {
            if( margin <= y && y < edges.rows - margin && margin <= x && x < edges.cols - margin )
            {
                if( edges.at<unsigned char>( y, x ) != 0 )
                {
                    if( y < miny )
                        miny = y;
                    if( y > maxy )
                        maxy = y;
                    if( x < minx )
                        minx = x;
                    if( x > maxx )
                        maxx = x;
                }
            }
            //else
            //{
                //edges.at<unsigned char>( y, x ) = 0;
            //}
        }
    }

    //const int margin = 3;
    miny -= 1;
    minx -= 1;
    maxy += 1;
    maxx += 1;
    if( !( 0 <= miny && miny <= maxy && maxy < edges.rows ) )
    {
        printf( "%d, %d\n", miny, maxy);
        exit( -1 );
    }
    if( !( 0 <= minx && minx <= maxx && maxx < edges.cols ) )
    {
        printf( "%d, %d\n", minx, maxx);
        exit( -1 );
    }

    return Rect( minx, miny, maxx - minx + 1, maxy - miny + 1 );
}

void ContourSegmentNetwork::detectByChamferMatching( ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects, bool visualize, Mat &visualization ) const
{
	bool verbose = false;
    /*
    vector<Point2f> trainPoints;
    for( size_t i=0;i<PASes.size();i++ )
    {
        trainPoints.push_back( PASes[i].descriptor.intersectPoint );
    }
    Rect trainBoundingRect = boundingRect( Mat( trainPoints ) );

    const double boundingRectScale = sqrt( 1.2 );
    trainBoundingRect.x -= trainBoundingRect.width * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.y -= trainBoundingRect.height * ( boundingRectScale - 1. ) / 2.;
    trainBoundingRect.width *= boundingRectScale;
    trainBoundingRect.height *= boundingRectScale;
*/

    //ChamferMatching chamferMatching( false );
    //ChamferMatching chamferMatching( true, 40 );
	ChamferMatching chamferMatching;

    //Mat edgesMat = calcEdges( baseImage );
    double t = (double)getTickCount();
    //Mat edgesMat = calcFilteredEdges();
    //assert( !baseEdges.empty() );
    //imwrite("baseEdges.png", baseEdges );
    //assert( boundingBox.area() != 0 );

    Mat edgesMat = calcObjectEdges( baseImage );

    Rect boundingBox = calcObjectBoundingBox( edgesMat );

    //imwrite("edges.png", edgesMat( trainBoundingRect ) );
    //imwrite("edges.png", edgesMat( boundingBox ) );
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("Filtering time: %f\n", t);

    t = (double)getTickCount();
    //IplImage edges = edgesMat( trainBoundingRect );
    IplImage edges = edgesMat( boundingBox );
    //chamferMatching.addTemplateFromImage( &edges );


    const float minAngle = -80;
    const float maxAngle = 80;
    const int nsteps = 16;
    const float stepAngle = ( maxAngle - minAngle ) / nsteps;
    //const float angleGranularity = M_PI / 16.;
    for( int step=0;step<=nsteps;step++ )
    {
    	float angle = minAngle + step*stepAngle;

    	Mat rotationMatrix = getRotationMatrix2D( Point2f( boundingBox.x + boundingBox.width / 2., boundingBox.y + boundingBox.height / 2. ), angle, 1. );
    	Mat rotatedImage;
    	warpAffine( baseImage, rotatedImage, rotationMatrix, baseImage.size() );
    	Mat rotatedEdges = calcObjectEdges( rotatedImage );
    	Rect box = calcObjectBoundingBox( rotatedEdges );

    	for( int i=box.y;i<box.y+box.height;i++ )
    	{
    		rotatedEdges.at<uchar>( i, box.x ) = 0;
    		rotatedEdges.at<uchar>( i, box.x + box.width - 1 ) = 0;
    	}
    	for( int j=box.x;j<box.x+box.width;j++ )
    	{
    		rotatedEdges.at<uchar>( box.y, j ) = 0;
    		rotatedEdges.at<uchar>( box.y + box.height - 1, j ) = 0;
    	}

    	//stringstream str;
    	//str << step << ".png";
    	//imwrite( str.str(), rotatedEdges( box ) );


    	IplImage edgesTemplate = rotatedEdges( box );
    	chamferMatching.addTemplateFromImage( &edgesTemplate );
    }
    //exit(-1);



    Mat testImageMat = testCSN.filterEdgesByLength( testCSN.calcEdges( testCSN.baseImage ) );
    //Mat wideTestImage = Mat( testImageMat.rows + 2*margin, testImageMat.cols + 2*margin, testImageMat.type() );
    //Mat tmpImage = wideTestImage( Rect(margin, margin, testImageMat.cols, testImageMat.rows) );
    //assert( tmpImage.rows == testImageMat.rows && tmpImage.cols == testImageMat.cols && tmpImage.type() == testImageMat.type() );
    //tmpImage = testCSN.calcEdges( testCSN.baseImage );

    for( int i=0;i<testImageMat.cols;i++ )
    {
        testImageMat.at<unsigned char>( 0, i ) = 0;
        testImageMat.at<unsigned char>( testImageMat.rows-1, i ) = 0;
    }

    for( int i=0;i<testImageMat.rows;i++ )
    {
        testImageMat.at<unsigned char>( i, 0 ) = 0;
        testImageMat.at<unsigned char>( i, testImageMat.cols - 1 ) = 0;
    }

    //imwrite("testImage.png", testImageMat);



    IplImage testImage = testImageMat;
    //IplImage testImage = wideTestImage;
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("Add template time: %f\n", t);


    t = (double)getTickCount();
    const int stepX = 1;
    const int stepY = 1;
    //SlidingWindowImageRange slidingWindowImageRange( trainBoundingRect.width, trainBoundingRect.height, stepX, stepY, 1, 1., 1. );
    SlidingWindowImageRange slidingWindowImageRange( testImageMat.cols, testImageMat.rows, stepX, stepY, 1, 1., 1. );
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    	printf("Sliding Window time: %f\n", t);


    const float orientation_weight = 0.5;
    t = (double)getTickCount();
    ChamferMatch chamferMatch = chamferMatching.matchEdgeImage( &testImage, slidingWindowImageRange, orientation_weight, 1 );

    ChamferMatch::ChamferMatches chamferMatches = chamferMatch.getMatches();
    t = ((double)getTickCount() - t)/getTickFrequency();
    if( verbose )
    {
    	printf("Chamfer time: %f\n", t);
    	printf("Size: %d\n", chamferMatches.size() );
    }
    assert( chamferMatches[0].tpl != 0 );
    assert( chamferMatches.size() == 1 );
    for( size_t i=0;i<chamferMatches.size();i++ )
    {
        //printf("%d\n", i );
        assert( chamferMatches[i].tpl != 0 );
        {
            //Rect rect( chamferMatches[i].offset.x, chamferMatches[i].offset.y, chamferMatches[i].tpl->size );
        	if( verbose )
        		printf("Object: [%d, %d, %d, %d]\n", chamferMatches[i].offset.x, chamferMatches[i].offset.y, chamferMatches[i].tpl->size.width, chamferMatches[i].tpl->size.height );

            //Rect rect( chamferMatches[i].offset, chamferMatches[i].tpl->size );
            CvPoint offset = chamferMatches[i].offset;
            CvPoint center = chamferMatches[i].tpl->center;
            CvPoint lu = cvPoint( offset.x - center.x, offset.y - center.y );
            //CvPoint lu = cvPoint( offset.x , offset.y );
            Rect rect( lu, chamferMatches[i].tpl->size );
            boundingRects.push_back( pair<Rect, double>( rect, -chamferMatches[i].cost ) );

            if( visualize )
            {
                //IplImage *weightsIpl = chamferMatching.visualize( &testImage, offset, chamferMatches[i].tpl, 0.5 );
                IplImage *weightsIpl;// = chamferMatching.visualize( &testImage, offset, chamferMatches[i].tpl, orientation_weight );
                Mat weights = weightsIpl;
                normalize( weights, weights, 255, 0, NORM_INF );
                visualization = visualizeBoundingRects( testCSN, boundingRects );

                const float eps = 1e-2;
                for( int y=0;y<weights.rows;y++ )
                {
                    for( int x=0;x<weights.cols;x++ )
                    {
                        if( weights.at<float>( y, x ) > eps )
                            visualization.at<Vec3b>( y, x ) = Vec3b( 0, weights.at<float>( y, x ), 0 );
                    }
                }
                cvReleaseImage( &weightsIpl );
            }
        }
    }



    //printf("The end!\n");

}

Mat ContourSegmentNetwork::visualizeBoundingRects( ContourSegmentNetwork &testCSN, vector< std::pair<Rect, double> > &boundingRects ) const
{
    //const Mat img1 = testCSN.drawAllSegments();
    //const Mat img2 = drawAllSegments();
    const Mat img1 = testCSN.filterEdgesByLength( testCSN.calcEdges( testCSN.baseImage ) );
    const Mat img2 = calcObjectEdges( baseImage );


    Size size(img1.cols + img2.cols, MAX(img1.rows, img2.rows));
    Mat drawImg(size, CV_MAKETYPE(img1.depth(), 3));
    drawImg.setTo(Scalar::all(0));
    Mat drawImg1 = drawImg(Rect(0, 0, img1.cols, img1.rows));
    cvtColor( img1, drawImg1, CV_GRAY2RGB );
    Mat drawImg2 = drawImg(Rect(img1.cols, 0, img2.cols, img2.rows));
    cvtColor( img2, drawImg2, CV_GRAY2RGB );

    for( int rectIdx=0;rectIdx<boundingRects.size();rectIdx++ )
    {
        Rect rect = boundingRects[rectIdx].first;
        //rectangle( drawImg1, rect, Scalar(0,255,0) );
    }
    //rectangle( drawImg2, boundingBox, Scalar(0,255,0) );

    return drawImg;
}

int ContourSegmentNetwork::drawSegment( Mat &image, int i, bool isDrawNeighborhood ) const
{
    Scalar segmentColor = Scalar( 255 );
    Scalar neighborColor = Scalar( 150 );

    int idx = i;
    if( idx<0 || idx>segmentsIndices.size() )
    {
        RNG rng( cvGetTickCount() );
        idx = rng( segmentsIndices.size() );
    }
    int segmentIndex = segmentsIndices[ idx ];
    if( segmentIndex<0 || segmentIndex>segments.size() )
        return segmentIndex;
    Segment segment = segments[ segmentIndex ];
    line( image, segment.back, segment.front, segmentColor );

    if( isDrawNeighborhood )
    {
        for( std::set<int>::iterator p=segment.neighborhood.begin(); p != segment.neighborhood.end(); p++ )
        {
            int k = segmentsIndices[ *p ];
            line( image, segments[ k ].back, segments[ k ].front, neighborColor );
        }
    }
    return idx;
}

Mat ContourSegmentNetwork::drawAllSegments() const
{
    Mat image = Mat( baseImage.rows, baseImage.cols, CV_8UC1, Scalar( 0 ) );
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        drawSegment( image, i, false );
    }
    return image;
}

int ContourSegmentNetwork::drawPAS( Mat &image, int i ) const
{
    Scalar firstColor = Scalar( 255 );
    Scalar secondColor = Scalar( 150 );

    int idx = i;
    if( idx<0 || idx>PASes.size() )
    {
        RNG rng( cvGetTickCount() );
        idx = rng( PASes.size() );
    }
    PAS pas = PASes[ idx ];
    line( image, segments[ segmentsIndices[pas.index1] ].back, segments[ segmentsIndices[pas.index1] ].front, firstColor );
    line( image, segments[ segmentsIndices[pas.index2] ].back, segments[ segmentsIndices[pas.index2] ].front, secondColor );
    return idx;
}

Mat ContourSegmentNetwork::drawAllPASes() const
{
    Mat image = Mat( baseImage.rows, baseImage.cols, CV_8UC3, Scalar( 0 ) );
    for( size_t i=0;i<PASes.size();i++ )
    {
        drawPAS( image, i );
    }
    return image;
}

Mat ContourSegmentNetwork::drawAllApproxCurves( ) const
{
    Mat image = Mat( baseImage.rows, baseImage.cols, CV_8UC1, Scalar( 0 ) );
    RNG rng (cvGetTickCount());
    for( size_t i=0;i<approxCurves.size();i++ )
    {
        Scalar curveColor = Scalar( 55 + rng(200) );
        for( size_t j=0;j<approxCurves[i].size()-1;j++ )
        {
            line( image, approxCurves[i][j], approxCurves[i][j+1], curveColor );
        }
    }
    return image;
}

vector<KeyPoint> ContourSegmentNetwork::PASesToKeypoints () const
{
    vector<KeyPoint> result;
    for( size_t i=0;i<PASes.size();i++ )
    {
        Point pt = PASes[i].descriptor.intersectPoint;
        //Point2f pt = PASes[i].descriptor.intersectPoint;
        //Point2f pt = PASes[i].location;

        //if( pt.x < 0 || pt.x >= baseImage.cols)
            //continue;
        //if( pt.y < 0 || pt.y >= baseImage.rows)
            //continue;

        assert( !(pt.x < 0 || pt.x >= baseImage.cols) );
        assert( !(pt.y < 0 || pt.y >= baseImage.rows) );
            //continue;

        int size = 1;
        KeyPoint keypt = KeyPoint( pt, size );

#ifdef OPENCV_PAS_BRANCH
        keypt.data = Mat( 1, 16, CV_32FC1 );
        Mat_<float>::iterator it = keypt.data.begin<float>();

        *(it++) = 2; //number of segments in feature;

        *(it++) = PASes[i].descriptor.orientation1;
        *(it++) = PASes[i].descriptor.orientation2;
        *(it++) = PASes[i].descriptor.length1;
        *(it++) = PASes[i].descriptor.length2;
        *(it++) = PASes[i].descriptor.rVector.x;
        *(it++) = PASes[i].descriptor.rVector.y;

        Segment segment = segments[ segmentsIndices[ PASes[i].index1 ] ];
        Segment segment1 = segment;
        *(it++) = segment.back.x;
        *(it++) = segment.back.y;
        *(it++) = segment.front.x;
        *(it++) = segment.front.y;

        segment = segments[ segmentsIndices[ PASes[i].index2 ] ];
        Segment segment2 = segment;
        *(it++) = segment.back.x;
        *(it++) = segment.back.y;
        *(it++) = segment.front.x;
        *(it++) = segment.front.y;


        Point2f vec1 = segment1.front - segment1.back;
        Point2f vec2 = segment2.front - segment2.back;

        //*(it++) = fabs( vec1.dot( vec2 ) );
        *(it++) = PASes[i].scale;
        assert( it == keypt.data.end<float>() );
#endif

        result.push_back( keypt );
    }

/*
    for( size_t i=0;i<segmentsIndices.size();i++ )
    {
        if( segmentsIndices[i] < 0)
            continue;
        Segment segment = segments[i];

        float minNorm = 32;
        if( segment.getNorm() < minNorm )
            continue;

        Point2f pt = (segment.back + segment.front) * 0.5;
        if( pt.x < 0 || pt.x >= baseImage.cols)
            continue;
        if( pt.y < 0 || pt.y >= baseImage.rows)
            continue;

        int size = 1;
        KeyPoint keypt = KeyPoint( pt, size );



        keypt.data = Mat( 1, 16, CV_32FC1 );
        Mat_<float>::iterator it = keypt.data.begin<float>();

        *(it++) = 1; //number of segments in feature
        *(it++) = segment.getNorm();
        *(it++) = segment.getOrientation();
        result.push_back( keypt );
    }
*/


    return result;
}

void PASDetector::detectImpl( const Mat& image, const Mat& mask, vector<KeyPoint>& keypoints ) const
{
    //assert( !mask.empty() );

    ContourSegmentNetwork csn;
    csn.calcContourSegmentNetwork( image );
    keypoints = csn.PASesToKeypoints();
}

void PASDescriptor::compute( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors ) const
{
    //assert( !keypoints.empty() );
    if( keypoints.empty() )
        return;
#ifdef OPENCV_PAS_BRANCH
    descriptors = Mat( keypoints.size(), keypoints[0].data.cols, CV_32F );
    for( size_t i=0; i<keypoints.size(); i++ )
    {
        Mat descriptor = descriptors.row( i );
        keypoints[i].data.row( 0 ).copyTo( descriptor );
    }
#endif
}
