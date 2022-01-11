//============================================================================
// Name        : Pcv3.cpp
// Author      : Ronny Haensch
// Version     : 1.0
// Copyright   : -
// Description : Camera calibration
//============================================================================

#include "Pcv3.h"
#include <math.h>

namespace pcv3 {

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
	std::vector<cv::Vec3f> new_points = points;
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
	t1(1, 1) = 1 / Sy;
	t2(0, 2) = -t_x;
	t2(1, 2) = -t_y;
	cv::Matx33f T = t1 * t2;
	return T;
    //return cv::Matx33f::eye();
}

/**
 * @brief get the conditioning matrix of given points
 * @param the points as matrix
 * @returns the condition matrix (already allocated)
 */
cv::Matx44f getCondition3D(const std::vector<cv::Vec4f> &points)
{
    // TO DO !!!
	float num_x = 0;
	float num_y = 0;
	float num_z = 0;
	for (int i = 0; i < points.size(); ++i)
	{
		num_x += points[i][0];
		num_y += points[i][1];
		num_z += points[i][2];
	}
	float t_x = num_x / points.size();
	float t_y = num_y / points.size();
	float t_z = num_z / points.size();
	std::vector<cv::Vec4f> new_points = points;
	for (int i = 0; i < points.size(); ++i)
	{
		new_points[i][0] = points[i][0] - t_x;
		new_points[i][1] = points[i][1] - t_y;
		new_points[i][2] = points[i][2] - t_z;
		new_points[i][3] = 1;
	}
	float sum_x = 0;
	float sum_y = 0;
	float sum_z = 0;
	for (int j = 0; j < new_points.size(); ++j)
	{
		sum_x += new_points[j][0];
		sum_y += new_points[j][1];
		sum_z += new_points[j][2];
	}
	float Sx = sum_x / new_points.size();
	float Sy = sum_y / new_points.size();
	float Sz = sum_z / new_points.size();
	cv::Matx44f t1 = cv::Matx44f::eye();
	cv::Matx44f t2 = cv::Matx44f::eye();
	t1(0, 0) = 1 / Sx;
	t1(1, 1) = 1 / Sy;
	t1(2, 2) = 1 / Sz;
	t2(0, 3) = -t_x;
	t2(1, 3) = -t_y;
	t2(2, 3) = -t_z;
	cv::Matx44f T = t1 * t2;
	return T;
    //return cv::Matx44f::eye();
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
 * @brief Applies a 3D transformation to an array of points
 * @param H Matrix representing the transformation
 * @param points Array of input points, each in homogeneous coordinates
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec4f> applyH_3D_points(const std::vector<cv::Vec4f>& points, const cv::Matx44f &H)
{
    std::vector<cv::Vec4f> result;

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
	for (int i = 0; i < points.size(); i++)
	{
		result.push_back(H * points[i]);
	}
    return result;
}



/**
 * @brief Define the design matrix as needed to compute projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_camera(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    // TO DO !!!
	int n = points2D.size();
	cv::Mat A = cv::Mat_<float>::zeros(2 * n, 12);
	for (int i = 0; i < n; i++)
	{
		A.at<float>(2 * i, 0) = -points2D[i][2] * points3D[i][0];
		A.at<float>(2 * i, 1) = -points2D[i][2] * points3D[i][1];
		A.at<float>(2 * i, 2) = -points2D[i][2] * points3D[i][2];
		A.at<float>(2 * i, 3) = -points2D[i][2] * points3D[i][3];

		A.at<float>(2 * i, 8) = points2D[i][0] * points3D[i][0];
		A.at<float>(2 * i, 9) = points2D[i][0] * points3D[i][1];
		A.at<float>(2 * i, 10) = points2D[i][0] * points3D[i][2];
		A.at<float>(2 * i, 11) = points2D[i][0] * points3D[i][3];

		A.at<float>(2 * i + 1, 4) = -points2D[i][2] * points3D[i][0];
		A.at<float>(2 * i + 1, 5) = -points2D[i][2] * points3D[i][1];
		A.at<float>(2 * i + 1, 6) = -points2D[i][2] * points3D[i][2];
		A.at<float>(2 * i + 1, 7) = -points2D[i][2] * points3D[i][3];

		A.at<float>(2 * i + 1, 8) = points2D[i][1] * points3D[i][0];
		A.at<float>(2 * i + 1, 9) = points2D[i][1] * points3D[i][1];
		A.at<float>(2 * i + 1, 10) = points2D[i][1] * points3D[i][2];
		A.at<float>(2 * i + 1, 11) = points2D[i][1] * points3D[i][3];

	}
	return A;
    //return cv::Mat_<float>(2*points2D.size(), 12);
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated projection matrix
 */
cv::Matx34f solve_dlt_camera(const cv::Mat_<float>& A)
{
    // TO DO !!!
	cv::SVD svd(A, cv::SVD::FULL_UV);
	cv::Matx34f P = cv::Matx34f::eye();
	P(0, 0) = svd.vt.at<float>(11, 0);
	P(0, 1) = svd.vt.at<float>(11, 1);
	P(0, 2) = svd.vt.at<float>(11, 2);
	P(0, 3) = svd.vt.at<float>(11, 3);
	P(1, 0) = svd.vt.at<float>(11, 4);
	P(1, 1) = svd.vt.at<float>(11, 5);
	P(1, 2) = svd.vt.at<float>(11, 6);
	P(1, 3) = svd.vt.at<float>(11, 7);
	P(2, 0) = svd.vt.at<float>(11, 8);
	P(2, 1) = svd.vt.at<float>(11, 9);
	P(2, 2) = svd.vt.at<float>(11, 10);
	P(2, 3) = svd.vt.at<float>(11, 11);
	//cv::resize(svd.vt.at<float>(11,),P,Size(4,3))
	return P;
    //return cv::Matx34f::eye();
}

/**
 * @brief Decondition a projection matrix that was estimated from conditioned point clouds
 * @param T_2D Conditioning matrix of set of 2D image points
 * @param T_3D Conditioning matrix of set of 3D object points
 * @param P Conditioned projection matrix that has to be un-conditioned (in-place)
 */
cv::Matx34f decondition_camera(const cv::Matx33f& T_2D, const cv::Matx44f& T_3D, const cv::Matx34f& P)
{
    // TO DO !!!
	cv::Matx33f T_2D_inv;
	cv::invert(T_2D, T_2D_inv);
	cv::Matx34f p = T_2D_inv * P * T_3D;
	return p;
    //return P;
}


/**
 * @brief Estimate projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The projection matrix to be computed
 */
cv::Matx34f calibrate(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    // TO DO !!!
	cv::Matx33f T_2D = getCondition2D(points2D);
	cv::Matx44f T_3D = getCondition3D(points3D);
	std::vector<cv::Vec3f> conditioned_2D = applyH_2D(points2D, T_2D, GEOM_TYPE_POINT);
	std::vector<cv::Vec4f> conditioned_3D = applyH_3D_points(points3D, T_3D);
	cv::Mat_<float> A = getDesignMatrix_camera(conditioned_2D, conditioned_3D);
	cv::Matx34f H_ = solve_dlt_camera(A);
	cv::Matx34f H = decondition_camera(T_2D, T_3D, H_);
	return H;
    //return cv::Matx34f::eye();
}



/**
 * @brief Extract and prints information about interior and exterior orientation from camera
 * @param P The 3x4 projection matrix, only "input" to this function
 * @param K Matrix for returning the computed internal calibration
 * @param R Matrix for returning the computed rotation
 * @param info Structure for returning the interpretation such as principal distance
 */
void interprete(const cv::Matx34f &P, cv::Matx33f &K, cv::Matx33f &R, ProjectionMatrixInterpretation &info)
{
    // TO DO !!!
	//cv::decomposeProjectionMatrix(P,K,R,C);
	cv::Matx33f M;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			M(i, j) = P(i, j);
		}
	}
/*
	float det = cv::determinant(M);
	cv::Matx33f scal = cv::Matx33f::eye();
	float length = sqrt(pow(M(2, 0), 2) + pow(M(2, 1), 2) + pow(M(2, 2), 2));
	float lamda;
	if (det > 0)
	{
		lamda = 1.0f / length;
	}
	else
	{
		lamda = -1.0f / length;
	}
	//scal(0, 0) = lamda;
	//scal(1, 1) = lamda;
	//M = lamda * M;

	//M = M * (1.0f / M(2,2));
*/
	cv::Matx33f Qx = cv::Matx33f::eye();
	float t_Qx = sqrt(pow(M(2, 1), 2) + pow(M(2, 2), 2));
	float s_Qx = -M(2, 1) / t_Qx;
	float c_Qx = M(2, 2) / t_Qx;
	Qx(1, 1) = c_Qx;
	Qx(2, 2) = c_Qx;
	Qx(2, 1) = s_Qx;
	Qx(1, 2) = -s_Qx;
	cv::Matx33f M_p;
	M_p = M * Qx;

	cv::Matx33f Qy = cv::Matx33f::eye();
	float t_Qy = sqrt(pow(M_p(2, 0), 2) + pow(M_p(2, 2), 2));
	float s_Qy = -M_p(2, 0) / t_Qy;
	float c_Qy = M_p(2, 2) / t_Qy;
	Qy(0, 0) = c_Qy;
	Qy(2, 2) = c_Qy;
	Qy(0, 2) = -s_Qy;
	Qy(2, 0) = s_Qy;
	cv::Matx33f M_pp;
	M_pp = M_p * Qy;

	cv::Matx33f Qz = cv::Matx33f::eye();
	float t_Qz = sqrt(pow(M_pp(1, 0), 2) + pow(M_pp(1, 1), 2));
	float s_Qz = -M_pp(1, 0) / t_Qz;
	float c_Qz = M_pp(1, 1) / t_Qz;
	Qz(0, 0) = c_Qz;
	Qz(1, 1) = c_Qz;
	Qz(0, 1) = -s_Qz;
	Qz(1, 0) = s_Qz;
	cv::Matx33f M_ppp;
	M_ppp = M_pp * Qz;//Calibration matrix

    // K = ...;
    cv::Matx33f Q;
    cv::Matx33f R_;
	cv::Matx33f _Qx;
	cv::Matx33f _Qy;
	cv::Matx33f _Qz;
    //cv::RQDecomp3x3(M,R_,Q,_Qx,_Qy,_Qz);

	K = M * Qx * Qy * Qz;
	//K = M_ppp;
	//K = R_;
	K = K * (1.0f / K(2,2));

    // R = ...;
	cv::Matx33f Qx_T;
	cv::Matx33f Qy_T;
	cv::Matx33f Qz_T;
	cv::transpose(Qx, Qx_T);
	cv::transpose(Qy, Qy_T);
	cv::transpose(Qz, Qz_T);
	R = Qz_T * Qy_T * Qx_T;
	//R=Q;

	//C
	cv::Matx31f C;
	cv::Matx33f M_inv;
	cv::Matx31f p4 = cv::Matx31f::zeros();
	p4(0, 0) = P(0, 3);
	p4(1, 0) = P(1, 3);
	p4(2, 0) = P(2, 3);
	cv::invert(M, M_inv);
	C = -M_inv * p4;

    float pi = 3.141592653;
    // Principal distance or focal length
    info.principalDistance = K(0,0);

    // Skew as an angle and in degrees
    //atan(-K(0,0)/K(0,1))*180/pi
    info.skew = atan(-K(0,0)/K(0,1))*180/pi;

    // Aspect ratio of the pixels
    info.aspectRatio = K(1,1)/K(0,0);

    // Location of principal point in image (pixel) coordinates
    info.principalPoint(0) = K(0,2);
    info.principalPoint(1) = K(1,2);

    // Camera rotation angle 1/3
    info.omega = atan(-R(2,1)/R(2,2))*180/pi;

    // Camera rotation angle 2/3
    info.phi = asin(R(2,0))*180/pi;

    // Camera rotation angle 3/3
    //(atan(-R(1,0)/R(0,0))+(pi))*180/pi;
    info.kappa = atan2(-R(1,0),R(0,0))*180/pi;

    // 3D camera location in world coordinates
    info.cameraLocation(0) = C(0, 0);
    info.cameraLocation(1) = C(1, 0);
    info.cameraLocation(2) = C(2, 0);

}




}
