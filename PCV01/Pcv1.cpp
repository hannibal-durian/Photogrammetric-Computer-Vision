//============================================================================
// Name        : Pcv1.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description :
//============================================================================

#include "Pcv1.h"

#include <stdexcept>

namespace pcv1 {


/**
 * @brief Convert a 2D point from Euclidean to homogeneous coordinates
 * @param p The point to convert (in Euclidean coordinates)
 * @returns The same point in homogeneous coordinates
 */
cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p)
{
    // TO DO !!!
	//std::cout << "point p0: " << p[0] << "^T" << std::endl;
	//std::cout << "point p1: " << p[1] << "^T" << std::endl;
	cv::Vec3f h(1.0f, 1.0f, 1.0f);
	//std::cout << "point oldh0: " << h[0] << "^T" << std::endl;
	//std::cout << "point oldh1: " << h[1] << "^T" << std::endl;
	//std::cout << "point oldh2: " << h[2] << "^T" << std::endl;
	h[0] = h[0] * p[0];
	//std::cout << "point h0: " << h[0] << "^T" << std::endl;
	h[1] = h[1] * p[1];
	//std::cout << "point h1: " << h[1] << "^T" << std::endl;
	return h;
	//return cv::Vec3f();
}

/**
 * @brief Convert a 2D point from homogeneous to Euclidean coordinates
 * @param p The point to convert in homogeneous coordinates
 * @returns The same point in Euclidean coordinates
 */
cv::Vec2f hom2eucl_point_2D(const cv::Vec3f& p)
{
    // TO DO !!!
	cv::Vec2b Euc(1.0f,1.0f);
	if (p[2] == 0)
	{
		Euc[0] = p[1];
		Euc[1] = p[2];
	}
	else
	{
		Euc[0] = p[0] / p[2];
		Euc[1] = p[1] / p[2];
	}
	return Euc;
    //return cv::Vec2f();
}


/**
 * @brief Calculates the joining line between two points (in 2D)
 * @param p1 First of the two points in homogeneous coordinates
 * @param p2 Second of the two points in homogeneous coordinates
 * @returns The joining line in homogeneous coordinates
*/
cv::Vec3f getConnectingLine_2D(const cv::Vec3f& p1, const cv::Vec3f& p2)
{
    // TO DO !!!
	//cv::Vec3f l(1.0f, 1.0f, 1.0f);
	cv::Vec3f l;
	l[0] = p1[1] * p2[2] - p2[1] * p1[2];
	l[1] = p1[2] * p2[0] - p2[2] * p1[0];
	l[2] = p1[0] * p2[1] - p2[0] * p1[1];
	return l;
    //return cv::Vec3f();
}

/**
 * @brief Generates a 2D translation matrix T defined by translation (dx, dy)^T
 * @param dx The translation in x-direction
 * @param dy the translation in y-direction
 * @returns The resulting translation matrix
 */
cv::Matx33f getTranslationMatrix_2D(float dx, float dy)
{
    // TO DO !!!
	cv::Matx33f T = cv::Matx33f::eye();
	T(0, 2) = T(0, 2) + dx;
	T(1, 2) = T(1, 2) + dy;
	//M = cv::Mat::eye(3, 3, CV_64F);
	//M.at<uchar>(0, 2) = M.at<uchar>(0, 2) + dx;
	//M.at<uchar>(1, 2) = M.at<uchar>(1, 2) + dy;
	//T = M;
	return T;
    //return cv::Matx33f();
}

/**
 * @brief Generates a 2D rotation matrix R defined by angle phi
 * @param phi The rotation angle in degree (!)
 * @returns The resulting rotation matrix
 */
cv::Matx33f getRotationMatrix_2D(float phi)
{
    // TO DO !!!
    float pi = 3.14159265358979323846264338328;
	cv::Matx33f R = cv::Matx33f::eye();
	R(0, 0) = cos((phi/180)*pi);
	R(0, 1) = -sin((phi/180)*pi);
	R(1, 0) = sin((phi/180)*pi);
	R(1, 1) = cos((phi/180)*pi);
	//std::cout << "Matrix R: " << R << "^T" << std::endl;
	//M.at<uchar>(0, 0) = cos(phi);
	//M.at<uchar>(0, 1) = -sin(phi);
	//M.at<uchar>(1, 0) = sin(phi);
	//M.at<uchar>(1, 1) = cos(phi);
	return R;
    //return cv::Matx33f();
}

/**
 * @brief Generates a 2D isotropic scaling matrix S defined by scaling factor lambda
 * @param lambda The scaling parameter
 * @returns The resulting scaling matrix
 */
cv::Matx33f getScalingMatrix_2D(float lambda)
{
    // TO DO !!!
	cv::Matx33f S = cv::Matx33f::eye();
	S(0, 0) = 1 * lambda;
	S(1, 1) = 1 * lambda;
	//S(2, 2) = 1 / lambda;
	//M.at<uchar>(2, 2) = 1/lambda;

	return S;
    //return cv::Matx33f();
}

/**
 * @brief Combines translation-, rotation-, and scaling-matrices to a single transformation matrix H
 * @details The returned transformation behaves as if objects were first transformed by T, then by R, and finally by S.
 * @param Translation matrix
 * @param Rotation matrix
 * @param Scaling matrix
 * @returns The combined homography
 */
cv::Matx33f getH_2D(const cv::Matx33f& T, const cv::Matx33f& R, const cv::Matx33f& S)
{
    // TO DO !!!
	cv::Matx33f H = cv::Matx33f::eye();
    H = S*R*T;
	//H(0, 2) = T(0, 2); //by T
	//H(1, 2) = T(1, 2);

	//H(0, 0) = R(0, 0);//by R
	//H(0, 1) = R(0, 1);
	//H(1, 0) = R(1, 0);
	//H(1, 1) = R(1, 1);

	//H(0, 0) = H(0, 0)*S(0, 0);//by S
	//H(1, 1) = H(1, 1)*S(1, 1);
	//std::cout << "Matrix H: " << H << "^T" << std::endl;
	return H;
    //return cv::Matx33f();
}

/**
 * @brief Applies a 2D transformation to an array of points or lines
 * @param H Matrix representing the transformation
 * @param geomObjects Array of input objects, each in homogeneous coordinates
 * @param type The type of the geometric objects, point or line. All are the same type.
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type)
{
    std::vector<cv::Vec3f> result;

    /******* Small std::vector cheat sheet ************************************/
    /*
     *   Number of elements in vector:                 a.size()
     *   Access i-th element (reading or writing):     a[i]
     *   Resize array:                                 a.resize(count);
     *   Append an element to an array:                a.push_back(element);
     *     \-> preallocate memory for e.g. push_back:  a.reserve(count);
     */
    /**************************************************************************/

    // TO DO !!!

    switch (type) {
        case GEOM_TYPE_POINT: {
            // TO DO !!!
			for (int i = 0; i < geomObjects.size(); i++)
			{
				//result[i] = H * geomObjects[i];
				result.push_back(H * geomObjects[i]);
			}
        } break;
        case GEOM_TYPE_LINE: {
            // TO DO !!!
			for (int i = 0; i < geomObjects.size(); i++)
			{
				//result[i] = H * geomObjects[i];
				cv::Matx33f N;
				cv::Matx33f M;
				cv::invert(H, N);
				cv::transpose(N, M);
				result.push_back(M * geomObjects[i]);
			}

        } break;
        default:
            throw std::runtime_error("Unhandled geometry type!");
    }
    return result;
}

/**
 * @brief Checks if a point is on a line
 * @param point The given point in homogeneous coordinates
 * @param line The given line in homogeneous coordinates
 * @param eps The used accuracy (allowed distance to still be considered on the line)
 * @returns Returns true if the point is on the line
 */
bool isPointOnLine_2D(const cv::Vec3f& point, const cv::Vec3f& line, float eps)
{
    // TO DO !!!
    //cv::Vec3f lT;
    //cv::transpose(line, lT);
	//cv::Matx13f P;
	//cv::Matx31f L;
	//P(0, 0) = point[0];
	//P(0, 1) = point[1];
	//P(0, 2) = point[2];
	//L(0, 0) = line[0];
	//L(1, 0) = line[1];
	//L(2, 0) = line[2]
    //cv::Matx<float,1,1> dis = P * L;
    //float dis = point*lT;
    //if(line[0]<0 and line[1]<0 and line[2]<0)
    //{
      //  line[0] = abs(line[0]);
        //line[1] = abs(line[1]);
        //line[2] = abs(line[2]);
    //}

    float dis = point[0] *line[0]+ point[1] * line[1]+ point[2] * line[2];
    if (abs(dis) < eps)
    {
        if ((point[0]!=0 and point[1]!=0 and point[2]==0) and (line[0]==0 and line[1]==0 and line[2]!=0) )
        {
            return true;
        }
        else
        {
            return true;
        }
    }
    return false;
}



/**
 * @brief Function loads input image, calls processing function and saves result (usually)
 * @param fname Path to input image
 */
void run(const std::string &fname){

    // window names
    std::string win1 = "Image";

    // load image as gray-scale, path in argv[1]
    std::cout << "Load image: start" << std::endl;
    cv::Mat inputImage;

    // TO DO !!!
    // inputimage = ???
	inputImage = cv::imread(fname);

    if (!inputImage.data){
        std::cout << "ERROR: image could not be loaded from " << fname << std::endl;
        std::cout << "Press enter to continue..." << std::endl;
        std::cin.get();
    }else
        std::cout << "Load image: done ( " << inputImage.rows << " x " << inputImage.cols << " )" << std::endl;

    // show input image
    cv::namedWindow( win1.c_str(), cv::WINDOW_AUTOSIZE );
    cv::imshow( win1.c_str(), inputImage );
    cv::waitKey(0);

    // the two given points as OpenCV matrices
    cv::Vec2f x(2.0f, 3.0f);
    cv::Vec2f y(-4.0f, 5.0f);

    // same points in homogeneous coordinates
    cv::Vec3f v1, v2;
    // TO DO !!!
    // define v1 as homogeneous version of x
	v1 = eucl2hom_point_2D(x);
    // define v2 as homogeneous version of y
	v2 = eucl2hom_point_2D(y);

    // print points
    std::cout << "point 1: " << v1.t() << "^T" << std::endl;
    std::cout << "point 2: " << v2.t() << "^T" << std::endl;
    std::cout << std::endl;

    // connecting line between those points in homogeneous coordinates
    cv::Vec3f line = getConnectingLine_2D(v1, v2);

    // print line
    std::cout << "joining line: " << line << "^T" << std::endl;
    std::cout << std::endl;

    // the parameters of the transformation
    int dx = 6;				// translation in x
    int dy = -7;			// translation in y
    float phi = 15;		// rotation angle in degree
    float lambda = 8;		// scaling factor

    // matrices for transformation
    // calculate translation matrix
    cv::Matx33f T = getTranslationMatrix_2D(dx, dy);
    // calculate rotation matrix
    cv::Matx33f R = getRotationMatrix_2D(phi);
    // calculate scale matrix
    cv::Matx33f S = getScalingMatrix_2D(lambda);
    // combine individual transformations to a homography
    cv::Matx33f H = getH_2D(T, R, S);

    // print calculated matrices
    std::cout << "Translation matrix: " << std::endl;
    std::cout << T << std::endl;
    std::cout << std::endl;
    std::cout << "Rotation matrix: " << std::endl;
    std::cout << R << std::endl;
    std::cout << std::endl;
    std::cout << "Scaling matrix: " << std::endl;
    std::cout << S << std::endl;
    std::cout << std::endl;
    std::cout << "Homography: " << std::endl;
    std::cout << H << std::endl;
    std::cout << std::endl;

    // transform first point x (and print it)
    cv::Vec3f v1_new = applyH_2D({v1}, H, GEOM_TYPE_POINT)[0];
    std::cout << "new point 1: " << v1_new << "^T" << std::endl;
    std::cout << "new point 1 (eucl): " << hom2eucl_point_2D(v1_new).t() << "^T" << std::endl;
    // transform second point y (and print it)
    cv::Vec3f v2_new = applyH_2D({v2}, H, GEOM_TYPE_POINT)[0];
    std::cout << "new point 2: " << v2_new << "^T" << std::endl;
    std::cout << "new point 2 (eucl): " << hom2eucl_point_2D(v2_new).t() << "^T" << std::endl;
    std::cout << std::endl;
    // transform joining line (and print it)
    cv::Vec3f line_new = applyH_2D({line}, H, GEOM_TYPE_LINE)[0];
    std::cout << "new line: " << line_new << "^T" << std::endl;
    std::cout << std::endl;

    // check if transformed points are still on transformed line
    bool xOnLine = isPointOnLine_2D(v1_new, line_new);
    bool yOnLine = isPointOnLine_2D(v2_new, line_new);
    if (xOnLine)
        std::cout << "first point lies still on the line *yay*" << std::endl;
    else
        std::cout << "first point does not lie on the line *oh oh*" << std::endl;

    if (yOnLine)
        std::cout << "second point lies still on the line *yay*" << std::endl;
    else
        std::cout << "second point does not lie on the line *oh oh*" << std::endl;

}


}
