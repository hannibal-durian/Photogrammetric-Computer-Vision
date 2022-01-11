//============================================================================
// Name        : Pcv2test.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description :
//============================================================================

#include "Pcv2.h"

namespace pcv2 {


/**
 * @brief get the conditioning matrix of given points
 * @param the points as matrix
 * @returns the condition matrix (already allocated)
 */
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f> &points)
{
    // TO DO !!!
	float num_x = 0;
	float num_y = 0;
	for (int i = 0; i < points.size(); ++i)
	{
		num_x += points[i][0];
		num_y += points[i][1];
	}
	float t_x = num_x / points.size();
	float t_y = num_y / points.size();
	std::vector<cv::Vec3f> new_points=points;
	for (int i = 0; i < points.size(); ++i)
	{
		new_points[i][0] = points[i][0] - t_x;
		new_points[i][1] = points[i][1] - t_y;
		new_points[i][2] = 1;
	}
	float sum_x = 0;
	float sum_y = 0;
	for (int j = 0; j < new_points.size(); ++j)
	{
		sum_x += new_points[j][0];
		sum_y += new_points[j][1];
	}
	float Sx = sum_x / new_points.size();
	float Sy = sum_y / new_points.size();
	cv::Matx33f t1 = cv::Matx33f::eye();
	cv::Matx33f t2 = cv::Matx33f::eye();
	t1(0, 0) = 1 / Sx;
	t1(0, 0) = 1 / Sy;
	t2(0, 2) = -t_x;
	t2(1, 2) = -t_y;
	cv::Matx33f T = t1 * t2;
	return T;
    //return cv::Matx33f::eye();
}


/**
 * @brief define the design matrix as needed to compute 2D-homography
 * @param conditioned_base first set of conditioned points x' --> x' = H * x
 * @param conditioned_attach second set of conditioned points x --> x' = H * x
 * @returns the design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_homography2D(const std::vector<cv::Vec3f> &conditioned_base, const std::vector<cv::Vec3f> &conditioned_attach)
{
    // TO DO !!!
	int n = conditioned_base.size();
	cv::Mat A = cv::Mat_<float>::zeros(2*n, 9);
	//int j = 0;
	for (int i = 0; i < n; i++)
	{
        A.at<float>(2*i, 0) = -conditioned_base[i][2] * conditioned_attach[i][0];
        A.at<float>(2*i, 1) = -conditioned_base[i][2] * conditioned_attach[i][1];
        A.at<float>(2*i, 2) = -conditioned_base[i][2] * conditioned_attach[i][2];

        A.at<float>(2*i, 6) = conditioned_base[i][0] * conditioned_attach[i][0];
        A.at<float>(2*i, 7) = conditioned_base[i][0] * conditioned_attach[i][1];
        A.at<float>(2*i, 8) = conditioned_base[i][0] * conditioned_attach[i][2];

        A.at<float>(2*i + 1, 3) = -conditioned_base[i][2] * conditioned_attach[i][0];
        A.at<float>(2*i + 1, 4) = -conditioned_base[i][2] * conditioned_attach[i][1];
        A.at<float>(2*i + 1, 5) = -conditioned_base[i][2] * conditioned_attach[i][2];

        A.at<float>(2*i + 1, 6) = conditioned_base[i][1] * conditioned_attach[i][0];
        A.at<float>(2*i + 1, 7) = conditioned_base[i][1] * conditioned_attach[i][1];
        A.at<float>(2*i + 1, 8) = conditioned_base[i][1] * conditioned_attach[i][2];
        //j += 2;
}
	return A;
    //return cv::Mat_<float>::zeros(8, 9);
}


/**
 * @brief solve homogeneous equation system by usage of SVD
 * @param A the design matrix
 * @returns solution of the homogeneous equation system
 */
cv::Matx33f solve_dlt_homography2D(const cv::Mat_<float> &A)
{
    // TO DO !!!
	cv::SVD svd(A, cv::SVD::FULL_UV);
	//std::vector<float> a = svd.w;
	//std::vector<float> d = svd.w;
	//std::sort(d.begin(), d.end());
	//int loc=0;
	//for (int i = 0; i < d.size(); i++)
	//{
	//	if (a[i] == d[0])
	//	{
	//		loc = i;
	//	}
	//}

	cv::Matx33f H = cv::Matx33f::eye();
	H(0, 0) = svd.vt.at<float>(8,0);
	H(0, 1) = svd.vt.at<float>(8,1);
	H(0, 2) = svd.vt.at<float>(8,2);
	H(1, 0) = svd.vt.at<float>(8,3);
	H(1, 1) = svd.vt.at<float>(8,4);
	H(1, 2) = svd.vt.at<float>(8,5);
	H(2, 0) = svd.vt.at<float>(8,6);
	H(2, 1) = svd.vt.at<float>(8,7);
	H(2, 2) = svd.vt.at<float>(8,8);
	//cv::resize(svd.vt.at<float>(8,),H,Size(3,3))
	return H;
    //return cv::Matx33f::eye();
}


/**
 * @brief decondition a homography that was estimated from conditioned point clouds
 * @param T_base conditioning matrix T' of first set of points x'
 * @param T_attach conditioning matrix T of second set of points x
 * @param H conditioned homography that has to be un-conditioned (in-place)
 */
cv::Matx33f decondition_homography2D(const cv::Matx33f &T_base, const cv::Matx33f &T_attach, const cv::Matx33f &H)
{
    // TO DO !!!
	cv::Matx33f T_base_inv;
	cv::invert(T_base, T_base_inv);
	cv::Matx33f h = T_base_inv * H * T_attach;
    return h;
}


/**
 * @brief compute the homography
 * @param base first set of points x'
 * @param attach second set of points x
 * @returns homography H, so that x' = Hx
 */
cv::Matx33f homography2D(const std::vector<cv::Vec3f> &base, const std::vector<cv::Vec3f> &attach)
{
    // TO DO !!!
	cv::Matx33f T_base = getCondition2D(base);
	cv::Matx33f T_attach = getCondition2D(attach);
	std::vector<cv::Vec3f> conditioned_base = applyH_2D(base, T_base, GEOM_TYPE_POINT);
	std::vector<cv::Vec3f> conditioned_attach = applyH_2D(attach, T_attach, GEOM_TYPE_POINT);
	cv::Mat_<float> A = getDesignMatrix_homography2D(conditioned_base, conditioned_attach);
	cv::Matx33f H_ = solve_dlt_homography2D(A);
	cv::Matx33f H = decondition_homography2D(T_base, T_attach, H_);
	return H;
    //return cv::Matx33f::eye();
}



// Functions from exercise 1
// Reuse your solutions from the last exercise here

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
 * @brief Convert a 2D point from Euclidean to homogeneous coordinates
 * @param p The point to convert (in Euclidean coordinates)
 * @returns The same point in homogeneous coordinates
 */
cv::Vec3f eucl2hom_point_2D(const cv::Vec2f& p)
{
    // TO DO !!!
	cv::Vec3f h(1.0f, 1.0f, 1.0f);
	h[0] = h[0] * p[0];
	h[1] = h[1] * p[1];
	return h;
    //return cv::Vec3f();
}

}
