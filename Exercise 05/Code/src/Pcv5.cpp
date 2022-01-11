//============================================================================
// Name        : Pcv5.cpp
// Author      : Andreas Ley
// Version     : 1.0
// Copyright   : -
// Description : Bundle Adjustment
//============================================================================

#include "Pcv5.h"
#include <random>
#include <opencv2/features2d.hpp>
#include <cmath>

using namespace cv;
using namespace std;

namespace pcv5 {


    
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
    cv::Matx33f H_inv = H.inv();

    switch (type) {
        case GEOM_TYPE_POINT: {
            for (int i = 0; i < geomObjects.size(); ++i)
            {
             cv::Vec3f point = geomObjects[i];
             point = H * point;
             // point = H * point.t();
             result.push_back(point);
            }
        } break;
        case GEOM_TYPE_LINE: {
            for (int i = 0; i < geomObjects.size(); ++i)
            {
             cv::Vec3f line = geomObjects[i];
             line = H_inv.t() * line;
             // point = H*point.t();
             result.push_back(line);
            }
        } break;
        default:
            throw std::runtime_error("Unhandled geometry type!");
    }
    return result;
}

/**
 * @brief Get the conditioning matrix of given points
 * @param p The points as matrix
 * @returns The condition matrix
 */
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f>& points2D)
{
   // Definition of some variables
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float coordinate_x = 0.0f;
    float coordinate_y = 0.0f;
    cv::Vec3f point;
    cv::Matx33f T = cv::Matx33f::eye();   // Translation matrix T

    // Summation of the coordinates
    for (int i = 0; i < points2D.size(); ++i)
    {
     point = points2D[i];
     coordinate_x = point[0];
     coordinate_y = point[1];
     sum_x += coordinate_x;
     sum_y += coordinate_y;
    }

    // Calculation of the transformation
    float t_x = sum_x / points2D.size();
    float t_y = sum_y / points2D.size();

    // Calculate the transformed points
    T(0, 2) = -t_x;
    T(1, 2) = -t_y;
    std::vector<cv::Vec3f> points_new = pcv5::applyH_2D(points2D, T, GEOM_TYPE_POINT);

    cv::Vec3f point_new;
    float sum_x_new = 0.0f;
    float sum_y_new = 0.0f;
    for (int i = 0; i < points_new.size(); ++i)
    {
     point_new = points_new[i];
     coordinate_x = point_new[0];
     coordinate_y = point_new[1];
     sum_x_new += std::abs(coordinate_x);
     sum_y_new += std::abs(coordinate_y);
    }   
    // Calculation of the scaling
    float s_x = sum_x_new / points_new.size();
    float s_y = sum_y_new / points_new.size();

    // Condition matrix
    T(0, 0) = 1/s_x;
    T(1, 1) = 1/s_y;
    T(0, 2) = -t_x/s_x;
    T(1, 2) = -t_y/s_y;
    // std::cout << "Conditioning matrix: " << std::endl;
    // std::cout << T << std::endl;

    return T;
}


/**
 * @brief Applies a 3D transformation to an array of points
 * @param H Matrix representing the transformation
 * @param points Array of input points, each in homogeneous coordinates
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec4f> applyH_3D_points(const std::vector<cv::Vec4f>& geomObjects, const cv::Matx44f &H)
{
    std::vector<cv::Vec4f> result;

    for (int i = 0; i < geomObjects.size(); ++i)
    {
        cv::Vec4f point = geomObjects[i];
        point = H * point;
        // point = H * point.t();
        result.push_back(point);
    }

    return result;
}

/**
 * @brief Get the conditioning matrix of given points
 * @param p The points as matrix
 * @returns The condition matrix 
 */
cv::Matx44f getCondition3D(const std::vector<cv::Vec4f>& points3D)
{
    // Definition of some variables
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    float coordinate_x = 0.0f;
    float coordinate_y = 0.0f;
    float coordinate_z = 0.0f;

    cv::Vec4f point;

    cv::Matx44f T = cv::Matx44f::eye();   // Translation matrix T'

    // Summation of the coordinates
    for (int i = 0; i < points3D.size(); ++i)
    {
     point = points3D[i];
     coordinate_x = point[0];
     coordinate_y = point[1];
     coordinate_z = point[2];
     sum_x += coordinate_x;
     sum_y += coordinate_y;
     sum_z += coordinate_z;
    }

    // Calculation of the transformation
    float t_x = sum_x / points3D.size();
    float t_y = sum_y / points3D.size();
    float t_z = sum_z / points3D.size();

    // Calculate the transformed points
    T(0, 3) = -t_x;
    T(1, 3) = -t_y;
    T(2, 3) = -t_z;
    std::vector<cv::Vec4f> points_new = pcv5::applyH_3D_points(points3D, T);

    cv::Vec4f point_new;
    float sum_x_new = 0.0f;
    float sum_y_new = 0.0f;
    float sum_z_new = 0.0f;
    for (int i = 0; i < points_new.size(); ++i)
    {
     point_new = points_new[i];
     coordinate_x = point_new[0];
     coordinate_y = point_new[1];
     coordinate_z = point_new[2];
     sum_x_new += std::abs(coordinate_x);
     sum_y_new += std::abs(coordinate_y);
     sum_z_new += std::abs(coordinate_z);
    }   

    // Calculation of the scaling
    float s_x = sum_x_new / points_new.size();
    float s_y = sum_y_new / points_new.size();
    float s_z = sum_z_new / points_new.size();   

    // Condition matrix
    T(0, 0) = 1/s_x;
    T(1, 1) = 1/s_y;
    T(2, 2) = 1/s_z;
    T(0, 3) = -t_x/s_x;
    T(1, 3) = -t_y/s_y;
    T(2, 3) = -t_z/s_z;
    // std::cout << "Conditioning matrix: " << std::endl;
    // std::cout << T << std::endl;

    return T;
}






/**
 * @brief Define the design matrix as needed to compute projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_camera(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    // Definition of some variables
    cv::Mat_<float> A_i = cv::Mat_<float>::zeros(2, 12);
    cv::Mat A;
    cv::Vec3f image_point;
    cv::Vec4f object_point;

    // Loop the points
    for (int i = 0; i < points3D.size(); ++i)
    {
     image_point = points2D[i];
     object_point = points3D[i];

     float image_u = image_point[0];
     float image_v = image_point[1];
     float image_w = image_point[2];

     float object_x = object_point[0];
     float object_y = object_point[1];
     float object_z = object_point[2];
     float object_w = object_point[3];

     // Calculation of the design matrix
     A_i(0, 0) = -image_w * object_x;
     A_i(0, 1) = -image_w * object_y;
     A_i(0, 2) = -image_w * object_z;
     A_i(0, 3) = -image_w * object_w;

     A_i(0, 8) = image_u * object_x;
     A_i(0, 9) = image_u * object_y;
     A_i(0, 10) = image_u * object_z;
     A_i(0, 11) = image_u * object_w;

     A_i(1, 4) = -image_w * object_x;
     A_i(1, 5) = -image_w * object_y;
     A_i(1, 6) = -image_w * object_z;
     A_i(1, 7) = -image_w * object_w;

     A_i(1, 8) = image_v * object_x;
     A_i(1, 9) = image_v * object_y;
     A_i(1, 10) = image_v * object_z;
     A_i(1, 11) = image_v * object_w;
     // Concatate the matrices 
     A.push_back(A_i);
    }
    // std::cout << "Design matrix: " << std::endl;
    // std::cout << A << std::endl;
    return A;
}

/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated projection matrix
 */
cv::Matx34f solve_dlt_camera(const cv::Mat_<float>& A)
{
    // Definition of some variables
    cv::Mat new_A = A.clone();
    cv::Mat U, W, V_T;
    cv::Matx34f P_attach = cv::Matx34f::eye();

    // SVD Decomposition
    cv::SVD svd(new_A, SVD::FULL_UV);

    // U matrix
    U = svd.u;
    // Singular values
    W = svd.w;
    // V_T matrix
    V_T = svd.vt;

    // Reshape
    V_T = V_T.t();

    int last_one = V_T.cols - 1;

    P_attach(0, 0) = V_T.at<float>(0, last_one);
    P_attach(0, 1) = V_T.at<float>(1, last_one);
    P_attach(0, 2) = V_T.at<float>(2, last_one);
    P_attach(0, 3) = V_T.at<float>(3, last_one);

    P_attach(1, 0) = V_T.at<float>(4, last_one);
    P_attach(1, 1) = V_T.at<float>(5, last_one);
    P_attach(1, 2) = V_T.at<float>(6, last_one);
    P_attach(1, 3) = V_T.at<float>(7, last_one);

    P_attach(2, 0) = V_T.at<float>(8, last_one);
    P_attach(2, 1) = V_T.at<float>(9, last_one);
    P_attach(2, 2) = V_T.at<float>(10, last_one);
    P_attach(2, 3) = V_T.at<float>(11, last_one);

    // std::cout << "Projection matrix: " << std::endl;
    // std::cout << P_attach << std::endl;

    return P_attach;
}

/**
 * @brief Decondition a projection matrix that was estimated from conditioned point clouds
 * @param T_2D Conditioning matrix of set of 2D image points
 * @param T_3D Conditioning matrix of set of 3D object points
 * @param P Conditioned projection matrix that has to be un-conditioned (in-place)
 */
cv::Matx34f decondition_camera(const cv::Matx33f& T_2D, const cv::Matx44f& T_3D, const cv::Matx34f& P)
{
    // Definition of some variables
    cv::Matx33f T_inv = T_2D.inv();
    cv::Matx34f P_new = cv::Matx34f::zeros();

    // Calculate the results
    P_new = T_inv * P * T_3D;

    // std::cout << "Deconditioned projection matrix: " << std::endl;
    // std::cout << P_new << std::endl;

    return P_new;
}

/**
 * @brief Estimate projection matrix
 * @param points2D Set of 2D points within the image
 * @param points3D Set of 3D points at the object
 * @returns The projection matrix to be computed
 */
cv::Matx34f calibrate(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D)
{
    // Conditioning
    cv::Matx33f T_attach = pcv5::getCondition2D(points2D);
    cv::Matx44f T_base = pcv5::getCondition3D(points3D);
    std::vector<cv::Vec3f> points2D_Con = pcv5::applyH_2D(points2D, T_attach, GEOM_TYPE_POINT);
    std::vector<cv::Vec4f> points3D_Con = pcv5::applyH_3D_points(points3D, T_base);

    // Get Design matrix
    cv::Mat A = pcv5::getDesignMatrix_camera(points2D_Con, points3D_Con);

    // Get Projection matrix
    cv::Matx34f P_attach = pcv5::solve_dlt_camera(A);

    // Deconditioning of the homography matrix
    cv::Matx34f P_new = pcv5::decondition_camera(T_attach, T_base, P_attach);

    // std::cout << "Result: " << std::endl;
    // std::cout << P_new << std::endl;

    return P_new;
}

/**
 * @brief Extract and prints information about interior and exterior orientation from camera
 * @param P The 3x4 projection matrix
 * @param K Matrix for returning the computed internal calibration
 * @param R Matrix for returning the computed rotation
 * @param info Structure for returning the interpretation such as principal distance
 */
void interprete(const cv::Matx34f &P, cv::Matx33f &K, cv::Matx33f &R, ProjectionMatrixInterpretation &info)
{
    // Definition of some variables
    cv::Matx33f M = cv::Matx33f::zeros();
    cv::Matx33f Qx;
    cv::Matx33f Qy;
    cv::Matx33f Qz;
    cv::Matx41f C;

    // Get M matrix
    M(0, 0) = P(0, 0);
    M(0, 1) = P(0, 1);
    M(0, 2) = P(0, 2);
    M(1, 0) = P(1, 0);
    M(1, 1) = P(1, 1);
    M(1, 2) = P(1, 2);
    M(2, 0) = P(2, 0);
    M(2, 1) = P(2, 1);
    M(2, 2) = P(2, 2);

    cv::RQDecomp3x3(M, K, R);
    // Normalization of K
    cv::Matx33f K_con = K * (1.0f / K(2,2));

    // SVD calculate C
    cv::Mat U, W, V_T;
    // SVD Decomposition
    cv::SVD svd(P, SVD::FULL_UV);

    // U matrix
    U = svd.u;
    // Singular values
    W = svd.w;
    // V_T matrix
    V_T = svd.vt;

    // Reshape
    V_T = V_T.t();

    int last_one = V_T.cols - 1;

    C(0, 0) = V_T.at<float>(0, last_one);
    C(1, 0) = V_T.at<float>(1, last_one);
    C(2, 0) = V_T.at<float>(2, last_one);
    C(3, 0) = V_T.at<float>(3, last_one);

    std::cout << C << std::endl;

    // Principal distance or focal length
    info.principalDistance = K_con(0, 0);
    
    // Skew as an angle and in degrees
    info.skew = atan(-K_con(0, 0)/K_con(0, 1))*180/M_PI;
    
    // Aspect ratio of the pixels
    info.aspectRatio = K_con(1, 1)/K_con(0, 0);
    
    // Location of principal point in image (pixel) coordinates
    info.principalPoint(0) = K_con(0, 2);
    info.principalPoint(1) = K_con(1, 2);
    
    // Camera rotation angle 1/3
    info.omega = atan(-R(2, 1)/R(2, 2))*180/M_PI;
    
    // Camera rotation angle 2/3
    info.phi = asin(R(2, 0))*180/M_PI;
    
    // Camera rotation angle 3/3
    info.kappa = atan(-R(1, 0)/R(0, 0))*180/M_PI;
    if(info.kappa < 0)
        info.kappa = 180 - std::abs(info.kappa);
    
    // 3D camera location in world coordinates
    info.cameraLocation(0) = C(0, 0)/C(3, 0);
    info.cameraLocation(1) = C(1, 0)/C(3, 0);
    info.cameraLocation(2) = C(2, 0)/C(3, 0);
}





/**
 * @brief Define the design matrix as needed to compute fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_fundamental(const std::vector<cv::Vec3f>& p1_conditioned, const std::vector<cv::Vec3f>& p2_conditioned)
{
    // Definition of some variables
    cv::Mat_<float> A_i = cv::Mat_<float>::zeros(1, 9);
    cv::Mat A;
    cv::Vec3f first_point;
    cv::Vec3f second_point;

    // Loop the points
    for (int i = 0; i < p1_conditioned.size(); ++i)
    {
     first_point = p1_conditioned[i];
     second_point = p2_conditioned[i];

     float first_x = first_point[0];
     float first_y = first_point[1];
     float first_z = first_point[2];

     float second_x = second_point[0];
     float second_y = second_point[1];
     float second_z = second_point[2];

     // Calculation of the design matrix
     A_i(0, 0) = first_x * second_x;
     A_i(0, 1) = first_y * second_x;
     A_i(0, 2) = first_z * second_x;

     A_i(0, 3) = first_x * second_y;
     A_i(0, 4) = first_y * second_y;
     A_i(0, 5) = first_z * second_y;

     A_i(0, 6) = first_x * second_z;
     A_i(0, 7) = first_y * second_z;
     A_i(0, 8) = first_z * second_z;

     // Concatate the matrices 
     A.push_back(A_i);
    }

    // std::cout << "Design matrix: " << std::endl;
    // std::cout << A << std::endl;

    return A;
}



/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated fundamental matrix
 */
cv::Matx33f solve_dlt_fundamental(const cv::Mat_<float>& A)
{
    // Definition of some variables
    cv::Mat new_A = A.clone();
    cv::Mat U, W, V_T;
    cv::Matx33f F_attach = cv::Matx33f::eye();

    // SVD Decomposition
    cv::SVD svd(new_A, SVD::FULL_UV);

    // U matrix
    U = svd.u;
    // Singular values
    W = svd.w;
    // std::cout << W << std::endl;
    // V_T matrix
    V_T = svd.vt;
    

    // Reshape
    V_T = V_T.t();
    //std::cout << V_T << std::endl;

    int last_one = V_T.cols - 1;

    F_attach(0, 0) = V_T.at<float>(0, last_one);
    F_attach(0, 1) = V_T.at<float>(1, last_one);
    F_attach(0, 2) = V_T.at<float>(2, last_one);

    F_attach(1, 0) = V_T.at<float>(3, last_one);
    F_attach(1, 1) = V_T.at<float>(4, last_one);
    F_attach(1, 2) = V_T.at<float>(5, last_one);

    F_attach(2, 0) = V_T.at<float>(6, last_one);
    F_attach(2, 1) = V_T.at<float>(7, last_one);
    F_attach(2, 2) = V_T.at<float>(8, last_one);


    // std::cout << "Estimated fundamental matrix: " << std::endl;
    // std::cout << F_attach << std::endl;

    return F_attach;
}


/**
 * @brief Enforce rank of 2 on fundamental matrix
 * @param F The matrix to be changed
 * @return The modified fundamental matrix
 */
cv::Matx33f forceSingularity(const cv::Matx33f& F)
{
    // Definition of some variables
    cv::Mat U, W, V_T;
    cv::Matx33f F_attach = cv::Matx33f::eye();
    cv::Matx33f W_new = cv::Matx33f::eye();
    cv::Matx33f U_new, V_T_new;

    // SVD Decomposition
    cv::SVD svd(F, SVD::FULL_UV);

    // U matrix
    U = svd.u;
    // Singular values
    W = svd.w;
    // V_T matrix
    V_T = svd.vt;

    for (int i = 0; i < U.rows; ++i)
    {
     for (int j = 0; j < U.cols; ++j)
     {
      U_new(i, j) = U.at<float>(i, j);
      V_T_new(i, j) = V_T.at<float>(i, j);
     }
    }

    W_new(0, 0) = W.at<float>(0, 0);
    W_new(1, 1) = W.at<float>(1, 0);
    W_new(2, 2) = 0;

    F_attach = U_new * W_new * V_T_new;

    // std::cout << "Estimated fundamental matrix after singularity constrains: " << std::endl;
    // std::cout << F_attach << std::endl;

    return F_attach;
}

/**
 * @brief Decondition a fundamental matrix that was estimated from conditioned points
 * @param T1 Conditioning matrix of set of 2D image points
 * @param T2 Conditioning matrix of set of 2D image points
 * @param F Conditioned fundamental matrix that has to be un-conditioned
 * @return Un-conditioned fundamental matrix
 */
cv::Matx33f decondition_fundamental(const cv::Matx33f& T1, const cv::Matx33f& T2, const cv::Matx33f& F)
{
    // Definition of some variables
    cv::Matx33f T_tran = T2.t();
    cv::Matx33f F_new = cv::Matx33f::zeros();

    // Calculate the results
    F_new = T_tran * F * T1;

    // std::cout << "Deconditioned fundamental matrix: " << std::endl;
    // std::cout << F_new << std::endl;
    return F_new;
}


/**
 * @brief Compute the fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns	the estimated fundamental matrix
 */
cv::Matx33f getFundamentalMatrix(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{
    // Conditioning
    cv::Matx33f T1 = pcv5::getCondition2D(p1);
    cv::Matx33f T2 = pcv5::getCondition2D(p2);
    std::vector<cv::Vec3f> p1_Con = pcv5::applyH_2D(p1, T1, GEOM_TYPE_POINT);
    std::vector<cv::Vec3f> p2_Con = pcv5::applyH_2D(p2, T2, GEOM_TYPE_POINT);

    // Get Design matrix
    cv::Mat A = pcv5::getDesignMatrix_fundamental(p1_Con, p2_Con);

    // Get Fundamental matrix
    cv::Matx33f F_attach = pcv5::solve_dlt_fundamental(A);

    // Apply singularity constrain
    F_attach = pcv5::forceSingularity(F_attach);

    // Deconditioning of the Fundamental matrix
    cv::Matx33f F_new = pcv5::decondition_fundamental(T1, T2, F_attach);

    // std::cout << "Result: " << std::endl;
    // std::cout << F_new << std::endl;

    return F_new;
}



/**
 * @brief Calculate geometric error of estimated fundamental matrix for a single point pair
 * @details Implement the "Sampson distance"
 * @param p1		first point
 * @param p2		second point
 * @param F		fundamental matrix
 * @returns geometric error
 */
float getError(const cv::Vec3f& p1, const cv::Vec3f& p2, const cv::Matx33f& F)
{
    float d;
    d = (((p2.t()*F*p1) * (p2.t()*F*p1)) / ((F*p1)[0]*(F*p1)[0] + (F*p1)[1]*(F*p1)[1] + (F.t()*p2)[0]*(F.t()*p2)[0] + (F.t()*p2)[1]*(F.t()*p2)[1]))[0];
    // std::cout << "Sampson error: " << std::endl;
    // std::cout << d << std::endl;
    return d;
}

/**
 * @brief Calculate geometric error of estimated fundamental matrix for a set of point pairs
 * @details Implement the mean "Sampson distance"
 * @param p1		first set of points
 * @param p2		second set of points
 * @param F		fundamental matrix
 * @returns geometric error
 */
float getError(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F)
{
    float d = 0.0f;
    cv::Vec3f first_point;
    cv::Vec3f second_point;

    // Calculate the summation
    for (int i = 0; i < p1.size(); ++i)
    {
     first_point = p1[i];
     second_point = p2[i];
     d += pcv5::getError(first_point, second_point, F);;    
    }

    // Calculate the average
    d = d / p1.size();
    // std::cout << "Average ampson error: " << std::endl;
    // std::cout << d << std::endl;
    return d;
}

/**
 * @brief Count the number of inliers of an estimated fundamental matrix
 * @param p1		first set of points
 * @param p2		second set of points
 * @param F		fundamental matrix
 * @param threshold Maximal "Sampson distance" to still be counted as an inlier
 * @returns		Number of inliers
 */
unsigned countInliers(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F, float threshold)
{
    unsigned count = 0;
    // Count the number of inliers of an estimated fundamental matrix
    cv::Vec3f first_point;
    cv::Vec3f second_point;
    float Sampson_distance;
    for (int i = 0; i < p1.size(); ++i)
    {
     first_point = p1[i];
     second_point = p2[i];
     Sampson_distance = pcv5::getError(first_point, second_point, F);
     if (Sampson_distance < threshold)
        count++;
    }
    // std::cout << "Number of inliers: " << std::endl;
    // std::cout << count << std::endl;
    return count;
}


/**
 * @brief Estimate the fundamental matrix robustly using RANSAC
 * @details Use the number of inliers as the score
 * @param p1 first set of points
 * @param p2 second set of points
 * @param numIterations How many subsets are to be evaluated
 * @param threshold Maximal "Sampson distance" to still be counted as an inlier
 * @returns The fundamental matrix
 */
cv::Matx33f estimateFundamentalRANSAC(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, unsigned numIterations, float threshold)
{
    // The length of subsets
    const unsigned subsetSize = 8;

    std::vector<cv::Vec3f> set1;
    std::vector<cv::Vec3f> set2;

    
    // Random number generater
    std::mt19937 rng;
    std::uniform_int_distribution<unsigned> uniformDist(0, p1.size()-1);
    // Draw a random point index with unsigned index = uniformDist(rng);

    unsigned inliers = 0;
    unsigned max = 0;
    int count = 0;
    cv::Matx33f F_final;

    while (count < numIterations)
    {
     set1.clear();
     set2.clear();
     // select random 8 points subsets
     for (int j = 0; j < subsetSize; ++j)
     {
      unsigned index = uniformDist(rng);
      set1.push_back(p1[index]);
      set2.push_back(p2[index]);
     }
     // estimate model from subset
     cv::Matx33f F = pcv5::getFundamentalMatrix(set1, set2);
     // count the inliers
     inliers = pcv5::countInliers(p1, p2, F, threshold);
     if (inliers > max)
     {
      F_final = F;
      max = inliers;
     }
     count++;
    }
    
    return F_final;
}








/**
 * @brief Computes the relative pose of two cameras given a list of point pairs and the camera's internal calibration.
 * @details The first camera is assumed to be in the origin, so only the external calibration of the second camera is computed. The point pairs are assumed to contain no outliers.
 * @param p1 Points in first image
 * @param p2 Points in second image
 * @param K Internal calibration matrix
 * @returns External calibration matrix of second camera
 */
cv::Matx44f computeCameraPose(const cv::Matx33f &K, const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{
    // Matrix for calculation of P
    cv::Matx34f M;
    M <<  1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0;
    cv::Matx44f H = cv::Matx44f::eye();
    // Projection Matrix of 1. camera
    cv::Matx34f P_first = K * (M * H);
    cout << P_first << endl;

    // The result of calculation for fundamental matrix is E
    cv::Matx33f F = getFundamentalMatrix(p1, p2);
    cv::Matx33f E = K.t() * F * K;
    // svd for E
    // Definition of some variables
    cv::Mat U, D, V_T;
    cv::Matx33f U_new, V_T_new;

    // SVD Decomposition
    cv::SVD svd(E, SVD::FULL_UV);

    // U matrix
    U = svd.u;
    float detU = cv::determinant(U);
    bool rightHanded = detU > 0.0f;
    if (!rightHanded)
    {
        for (int i = 0; i < U.rows; ++i)
        {
            for (int j = 0; j < U.cols; ++j)
            {
                U.at<float>(i, j) = -U.at<float>(i, j);
            }
        } 
    }
    // Singular values
    D = svd.w;
    // V_T matrix
    V_T = svd.vt;
    float detV = cv::determinant(V_T);
    rightHanded = detV > 0.0f;
    if (!rightHanded)
    {
        for (int i = 0; i < V_T.rows; ++i)
        {
            for (int j = 0; j < V_T.cols; ++j)
            {
                V_T.at<float>(i, j) = -V_T.at<float>(i, j);
            }
        } 
    }
    // Matrix W and W.t()
    cv::Matx33f W_1, W_2;
    W_1 <<  0, -1, 0,
            1,  0, 0,
            0,  0, 1;
    W_2 = W_1.t();


    // Matrix t and -t
    cv::Matx31f t_1, t_2;
    for (int i = 0; i < U.rows; ++i)
    {
      t_1(i, 0) = U.at<float>(i, 2);
    }
    for (int i = 0; i < t_1.rows; ++i)
    {
      t_2(i, 0) = -1 * t_1(i, 0);
    }
    // Matrix R
    for (int i = 0; i < U.rows; ++i)
    {
     for (int j = 0; j < U.cols; ++j)
     {
      U_new(i, j) = U.at<float>(i, j);
      V_T_new(i, j) = V_T.at<float>(i, j);
     }
    }
    cv::Matx33f R_1 = U_new * W_1 * V_T_new;
    cv::Matx33f R_2 = U_new * W_2 * V_T_new;

    // Concatentation
    cv::Matx44f H_1 = cv::Matx44f::eye();
    cv::Matx44f H_2 = cv::Matx44f::eye();
    cv::Matx44f H_3 = cv::Matx44f::eye();
    cv::Matx44f H_4 = cv::Matx44f::eye();
    cv::Matx34f P_1 = cv::Matx34f::eye();
    cv::Matx34f P_2 = cv::Matx34f::eye();
    cv::Matx34f P_3 = cv::Matx34f::eye();
    cv::Matx34f P_4 = cv::Matx34f::eye();
    std::vector<cv::Matx34f> P_vec;
    std::vector<cv::Matx44f> H_vec;
    // 1. R_1 + t_1
    for (int i = 0; i < R_1.rows; ++i)
    {
     for (int j = 0; j < R_1.cols; ++j)
     {
      H_1(i, j) = R_1(i, j);
     }
    }
    for (int i = 0; i < t_1.rows; ++i)
    {
      H_1(i, 3) = t_1(i, 0);
    }
    H_vec.push_back(H_1);
    P_1 = K * (M * H_1);
    P_vec.push_back(P_1);
    // 2. R_2 + t_2
    for (int i = 0; i < R_2.rows; ++i)
    {
     for (int j = 0; j < R_2.cols; ++j)
     {
      H_2(i, j) = R_2(i, j);
     }
    }
    for (int i = 0; i < t_2.rows; ++i)
    {
      H_2(i, 3) = t_2(i, 0);
    }
    H_vec.push_back(H_2);
    P_2 = K * (M * H_2);
    P_vec.push_back(P_2);
    // 3. R_1 + t_2
    for (int i = 0; i < R_1.rows; ++i)
    {
     for (int j = 0; j < R_1.cols; ++j)
     {
      H_3(i, j) = R_1(i, j);
     }
    }
    for (int i = 0; i < t_2.rows; ++i)
    {
      H_3(i, 3) = t_2(i, 0);
    }
    H_vec.push_back(H_3);
    P_3 = K * (M * H_3);
    P_vec.push_back(P_3);
    // 4. R_2 + t_1
    for (int i = 0; i < R_2.rows; ++i)
    {
     for (int j = 0; j < R_2.cols; ++j)
     {
      H_4(i, j) = R_2(i, j);
     }
    }
    for (int i = 0; i < t_1.rows; ++i)
    {
      H_4(i, 3) = t_1(i, 0);
    }
    H_vec.push_back(H_4);
    P_4 = K * (M * H_4);
    P_vec.push_back(P_4);

    for (int i = 0; i < H_vec.size(); ++i)
    {
       cout << H_vec[i] << endl;
    }

    // Select the most in front of the camera
    int max = 0;
    cv::Matx44f result_H;
    for (int i = 0; i < P_vec.size(); ++i)
    {
        int count = 0;
        cv::Matx34f P_second = P_vec[i];
        for (int j = 0; j < p1.size(); ++j)
        {
            cv::Vec4f result = pcv5::linearTriangulation(P_first, P_second, p1[j], p2[j]);
            if (result[2] / result[3] > 0.0f)
            {
                count++;
            }
        }
        if (count > max)
        {
            max = count;
            result_H = H_vec[i];
        }
    }

    return result_H;
}








/**
 * @brief Estimate the fundamental matrix robustly using RANSAC
 * @param p1 first set of points
 * @param p2 second set of points
 * @param numIterations How many subsets are to be evaluated
 * @returns The fundamental matrix
 */
cv::Matx34f estimateProjectionRANSAC(const std::vector<cv::Vec3f>& points2D, const std::vector<cv::Vec4f>& points3D, unsigned numIterations, float threshold)
{
    const unsigned subsetSize = 6;

    std::mt19937 rng;
    std::uniform_int_distribution<unsigned> uniformDist(0, points2D.size()-1);
    // Draw a random point index with unsigned index = uniformDist(rng);
    
    cv::Matx34f bestP;
    unsigned bestInliers = 0;
    
    std::vector<cv::Vec3f> points2D_subset;
    points2D_subset.resize(subsetSize);
    std::vector<cv::Vec4f> points3D_subset;
    points3D_subset.resize(subsetSize);
    for (unsigned iter = 0; iter < numIterations; iter++) {
        for (unsigned j = 0; j < subsetSize; j++) {
            unsigned index = uniformDist(rng);
            points2D_subset[j] = points2D[index];
            points3D_subset[j] = points3D[index];
        }
        
        cv::Matx34f P = calibrate(points2D_subset, points3D_subset);

        unsigned numInliers = 0;
        for (unsigned i = 0; i < points2D.size(); i++) {
            cv::Vec3f projected = P * points3D[i];
            if (projected(2) > 0.0f) // in front
                if ((std::abs(points2D[i](0) - projected(0)/projected(2)) < threshold) &&
                    (std::abs(points2D[i](1) - projected(1)/projected(2)) < threshold))
                    numInliers++;
        }

        if (numInliers > bestInliers) {
            bestInliers = numInliers;
            bestP = P;
        }
    }
    
    return bestP;
}


// triangulates given set of image points based on projection matrices
/*
P1	projection matrix of first image
P2	projection matrix of second image
x1	image point set of first image
x2	image point set of second image
return	triangulated object points
*/
cv::Vec4f linearTriangulation(const cv::Matx34f& P1, const cv::Matx34f& P2, const cv::Vec3f& x1, const cv::Vec3f& x2)
{
    // allocate memory for design matrix
    Mat_<float> A(4, 4);

    // create design matrix
    // first row	x1(0, i) * P1(2, :) - P1(0, :)
    A(0, 0) = x1(0) * P1(2, 0) - P1(0, 0);
    A(0, 1) = x1(0) * P1(2, 1) - P1(0, 1);
    A(0, 2) = x1(0) * P1(2, 2) - P1(0, 2);
    A(0, 3) = x1(0) * P1(2, 3) - P1(0, 3);
    // second row	x1(1, i) * P1(2, :) - P1(1, :)
    A(1, 0) = x1(1) * P1(2, 0) - P1(1, 0);
    A(1, 1) = x1(1) * P1(2, 1) - P1(1, 1);
    A(1, 2) = x1(1) * P1(2, 2) - P1(1, 2);
    A(1, 3) = x1(1) * P1(2, 3) - P1(1, 3);
    // third row	x2(0, i) * P2(3, :) - P2(0, :)
    A(2, 0) = x2(0) * P2(2, 0) - P2(0, 0);
    A(2, 1) = x2(0) * P2(2, 1) - P2(0, 1);
    A(2, 2) = x2(0) * P2(2, 2) - P2(0, 2);
    A(2, 3) = x2(0) * P2(2, 3) - P2(0, 3);
    // first row	x2(1, i) * P2(3, :) - P2(1, :)
    A(3, 0) = x2(1) * P2(2, 0) - P2(1, 0);
    A(3, 1) = x2(1) * P2(2, 1) - P2(1, 1);
    A(3, 2) = x2(1) * P2(2, 2) - P2(1, 2);
    A(3, 3) = x2(1) * P2(2, 3) - P2(1, 3);

    cv::SVD svd(A);
    Mat_<float> tmp = svd.vt.row(3).t();

    return cv::Vec4f(tmp(0), tmp(1), tmp(2), tmp(3));
}

std::vector<cv::Vec4f> linearTriangulation(const cv::Matx34f& P1, const cv::Matx34f& P2, const std::vector<cv::Vec3f>& x1, const std::vector<cv::Vec3f>& x2)
{
    std::vector<cv::Vec4f> result;
    result.resize(x1.size());
    for (unsigned i = 0; i < result.size(); i++)
        result[i] = linearTriangulation(P1, P2, x1[i], x2[i]);
    return result;
}



void BundleAdjustment::BAState::computeResiduals(float *residuals) const
{
    unsigned rIdx = 0;
    for (unsigned camIdx = 0; camIdx < m_cameras.size(); camIdx++) {
        const auto &calibState = m_internalCalibs[m_scene.cameras[camIdx].internalCalibIdx];
        const auto &cameraState = m_cameras[camIdx];
        
        // Compute 3x4 camera matrix (composition of internal and external calibration)
        // Internal calibration is calibState.K
        // External calibration is dropLastRow(cameraState.H)

        cv::Matx34f M;
        M <<  1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;
        cv::Matx44f H = cv::Matx44f::eye();
        // Projection Matrix 
        cv::Matx34f P = calibState.K * M * cameraState.H;
        
        for (const KeyPoint &kp : m_scene.cameras[camIdx].keypoints) {
            const auto &trackState = m_tracks[kp.trackIdx];
            // Using P, compute the homogeneous position of the track in the image (world space position is trackState.location)
            cv::Vec3f projection = P * trackState.location;
            
            // Compute the euclidean position of the track
            cv::Vec2f projection_eucl = pcv5::hom2eucl(projection);
            
            // Compute the residuals: the difference between computed position and real position (kp.location(0) and kp.location(1))
            // Compute and store the (signed!) residual in x direction multiplied by kp.weight
            residuals[rIdx++] = (kp.location(0) - projection_eucl[0]) * kp.weight;
            // Compute and store the (signed!) residual in y direction multiplied by kp.weight
            residuals[rIdx++] = (kp.location(1) - projection_eucl[1]) * kp.weight;
        }
    }
}

void BundleAdjustment::BAState::computeJacobiMatrix(JacobiMatrix *dst) const
{
    BAJacobiMatrix &J = dynamic_cast<BAJacobiMatrix&>(*dst);
    
    unsigned rIdx = 0;
    for (unsigned camIdx = 0; camIdx < m_cameras.size(); camIdx++) {
        const auto &calibState = m_internalCalibs[m_scene.cameras[camIdx].internalCalibIdx];   // cv::Matx33f K;
        const auto &cameraState = m_cameras[camIdx];      // cv::Matx44f H;
        
        for (const KeyPoint &kp : m_scene.cameras[camIdx].keypoints) {
            const auto &trackState = m_tracks[kp.trackIdx];      // cv::Vec4f location
            
            // calibState.K is the internal calbration
            // cameraState.H is the external calbration
            // trackState.location is the 3D location of the track in homogeneous coordinates

            // Compute the positions before and after the internal calibration (compare to slides).
            cv::Matx34f M;
            M <<  1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0;

            cv::Vec3f v = (M * cameraState.H) * trackState.location;     
            cv::Vec3f u = calibState.K * v;                           
            
            cv::Matx23f J_hom2eucl;

            // How do the euclidean image positions change when the homogeneous image positions change        
            J_hom2eucl(0, 0) = 1 / u[2];
            J_hom2eucl(0, 1) = 0;
            J_hom2eucl(0, 2) = -u[0] / (u[2] * u[2]);
            J_hom2eucl(1, 0) = 0;
            J_hom2eucl(1, 1) = 1/ u[2];
            J_hom2eucl(1, 2) = -u[1] / (u[2] * u[2]);
            
            
            cv::Matx33f du_dDeltaK;

            // How do homogeneous image positions change when the internal calibration is changed (the 3 update parameters)
            du_dDeltaK(0, 0) = v[0] * calibState.K(0, 0);
            du_dDeltaK(0, 1) = v[2] * calibState.K(0, 2);
            du_dDeltaK(0, 2) = 0;
            du_dDeltaK(1, 0) = v[1] * calibState.K(1, 1);
            du_dDeltaK(1, 1) = 0;
            du_dDeltaK(1, 2) = v[2] * calibState.K(1, 2);
            du_dDeltaK(2, 0) = 0;
            du_dDeltaK(2, 1) = 0;
            du_dDeltaK(2, 2) = 0;
            
            
            // Using the above (J_hom2eucl and du_dDeltaK), how do the euclidean image positions change when the internal calibration is changed (the 3 update parameters)
            // Remember to include the weight of the keypoint (kp.weight)
            J.m_rows[rIdx].J_internalCalib = (J_hom2eucl * du_dDeltaK) * kp.weight;      // 2*3
            
            // How do the euclidean image positions change when the tracks are moving in eye space/camera space (the vector "v" in the slides)?
            // cv::Matx<float, 2, 4> J_v2eucl = J.m_rows[rIdx].J_internalCalib * (M * cameraState.H);   // works like cv::Matx24f but the latter was not typedef-ed
            //cv::Matx<float, 2, 3> J_v2eucl = J_hom2eucl * calibState.K;
            //cv::Matx36f dv_dDeltaH;
            cv::Matx<float, 3, 6> dv_dDeltaH; // works like cv::Matx36f but the latter was not typedef-ed
            
            // How do tracks move in eye space (vector "v" in slides) when the parameters of the camera are changed
            dv_dDeltaH(0, 0) = 0;
            dv_dDeltaH(0, 1) = v[2];
            dv_dDeltaH(0, 2) = -v[1];
            dv_dDeltaH(0, 3) = trackState.location[3];
            dv_dDeltaH(0, 4) = 0;
            dv_dDeltaH(0, 5) = 0;
            dv_dDeltaH(1, 0) = -v[2];
            dv_dDeltaH(1, 1) = 0;
            dv_dDeltaH(1, 2) = v[0];
            dv_dDeltaH(1, 3) = 0;
            dv_dDeltaH(1, 4) = trackState.location[3];
            dv_dDeltaH(1, 5) = 0;
            dv_dDeltaH(2, 0) = v[1];
            dv_dDeltaH(2, 1) = -v[0];
            dv_dDeltaH(2, 2) = 0;
            dv_dDeltaH(2, 3) = 0;
            dv_dDeltaH(2, 4) = 0;
            dv_dDeltaH(2, 5) = trackState.location[3];
            
            // How do the euclidean image positions change when the external calibration is changed (the 6 update parameters)?
            // Remember to include the weight of the keypoint (kp.weight)
            J.m_rows[rIdx].J_camera = (J_hom2eucl * calibState.K * dv_dDeltaH) * kp.weight;     // 2*6
            
            
            // How do the euclidean image positions change when the tracks are moving in world space (the x, y, z, and w before the external calibration)?
            // The multiplication operator "*" works as one would suspect. You can use dropLastRow(...) to drop the last row of a matrix.
            cv::Matx<float, 2, 4> J_worldSpace2eucl = J_hom2eucl * calibState.K * (M * cameraState.H);
            
            
            // How do the euclidean image positions change when the tracks are changed. 
            // This is the same as above, except it should also include the weight of the keypoint (kp.weight)
            J.m_rows[rIdx].J_track = J_worldSpace2eucl * kp.weight;    // 2*4
            
            rIdx++;
        }
    }
}

void BundleAdjustment::BAState::update(const float *update, State *dst) const
{
    BAState &state = dynamic_cast<BAState &>(*dst);
    state.m_internalCalibs.resize(m_internalCalibs.size());
    state.m_cameras.resize(m_cameras.size());
    state.m_tracks.resize(m_tracks.size());
    
    unsigned intCalibOffset = 0;
    for (unsigned i = 0; i < m_internalCalibs.size(); i++) {
        state.m_internalCalibs[i].K = m_internalCalibs[i].K;
        /*
        * Modify the new internal calibration
        * 
        * m_internalCalibs[i].K is the old matrix, state.m_internalCalibs[i].K is the new matrix.
        * 
        */
        state.m_internalCalibs[i].K(0, 0) = state.m_internalCalibs[i].K(0, 0) * (1 + update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 0]); // is how much the focal length is supposed to change (scaled by the old focal length)
        state.m_internalCalibs[i].K(1, 1) = state.m_internalCalibs[i].K(1, 1) * (1 + update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 0]); // is how much the focal length is supposed to change (scaled by the old focal length)
        state.m_internalCalibs[i].K(0, 2) = state.m_internalCalibs[i].K(0, 2) * (1 + update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 1]); // is how much the principal point is supposed to shift in x direction (scaled by the old x position of the principal point)
        state.m_internalCalibs[i].K(1, 2) = state.m_internalCalibs[i].K(1, 2) * (1 + update[intCalibOffset + i * NumUpdateParams::INTERNAL_CALIB + 2]); // is how much the principal point is supposed to shift in y direction (scaled by the old y position of the principal point)
    }
    unsigned cameraOffset = intCalibOffset + m_internalCalibs.size() * NumUpdateParams::INTERNAL_CALIB;
    for (unsigned i = 0; i < m_cameras.size(); i++) {
        // state.m_cameras[i].H = m_cameras[i].H;
        /*
        * Compose the new matrix H
        * 
        * m_cameras[i].H is the old matrix, state.m_cameras[i].H is the new matrix.
        * 
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 0] rotation increment around the camera X axis (not world X axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 1] rotation increment around the camera Y axis (not world Y axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 2] rotation increment around the camera Z axis (not world Z axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 3] translation increment along the camera X axis (not world X axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 4] translation increment along the camera Y axis (not world Y axis)
        * update[cameraOffset + i * NumUpdateParams::CAMERA + 5] translation increment along the camera Z axis (not world Z axis)
        * 
        * use rotationMatrixX(...), rotationMatrixY(...), rotationMatrixZ(...), and translationMatrix
        * 
        */
        cv::Matx44f R = rotationMatrixX(update[cameraOffset + i * NumUpdateParams::CAMERA + 0])*
                        rotationMatrixY(update[cameraOffset + i * NumUpdateParams::CAMERA + 1])*
                        rotationMatrixZ(update[cameraOffset + i * NumUpdateParams::CAMERA + 2]);

        cv::Matx44f T = translationMatrix(update[cameraOffset + i * NumUpdateParams::CAMERA + 3],
                                          update[cameraOffset + i * NumUpdateParams::CAMERA + 4],
                                          update[cameraOffset + i * NumUpdateParams::CAMERA + 5]);

        state.m_cameras[i].H = R * T * m_cameras[i].H;

    }
    unsigned trackOffset = cameraOffset + m_cameras.size() * NumUpdateParams::CAMERA;
    for (unsigned i = 0; i < m_tracks.size(); i++) {
        state.m_tracks[i].location = m_tracks[i].location;     
        /*
        * Modify the new track location
        * 
        * m_tracks[i].location is the old location, state.m_tracks[i].location is the new location.
        * 
        * update[trackOffset + i * NumUpdateParams::TRACK + 0] increment of X
        * update[trackOffset + i * NumUpdateParams::TRACK + 1] increment of Y
        * update[trackOffset + i * NumUpdateParams::TRACK + 2] increment of Z
        * update[trackOffset + i * NumUpdateParams::TRACK + 3] increment of W
        */

        state.m_tracks[i].location(0) += update[trackOffset + i * NumUpdateParams::TRACK + 0];
        state.m_tracks[i].location(1) += update[trackOffset + i * NumUpdateParams::TRACK + 1];
        state.m_tracks[i].location(2) += update[trackOffset + i * NumUpdateParams::TRACK + 2];
        state.m_tracks[i].location(3) += update[trackOffset + i * NumUpdateParams::TRACK + 3];


        // Renormalization to length one
        float len = std::sqrt(state.m_tracks[i].location.dot(state.m_tracks[i].location));
        state.m_tracks[i].location *= 1.0f / len;
    }
}






/************************************************************************************************************/
/************************************************************************************************************/
/***************************                                     ********************************************/
/***************************    Nothing to do below this point   ********************************************/
/***************************                                     ********************************************/
/************************************************************************************************************/
/************************************************************************************************************/




BundleAdjustment::BAJacobiMatrix::BAJacobiMatrix(const Scene &scene)
{
    unsigned numResidualPairs = 0;
    for (const auto &camera : scene.cameras)
        numResidualPairs += camera.keypoints.size();
    
    m_rows.reserve(numResidualPairs);
    for (unsigned camIdx = 0; camIdx < scene.cameras.size(); camIdx++) {
        const auto &camera = scene.cameras[camIdx];
        for (unsigned kpIdx = 0; kpIdx < camera.keypoints.size(); kpIdx++) {
            m_rows.push_back({});
            m_rows.back().internalCalibIdx = camera.internalCalibIdx;
            m_rows.back().cameraIdx = camIdx;
            m_rows.back().keypointIdx = kpIdx;
            m_rows.back().trackIdx = camera.keypoints[kpIdx].trackIdx;
        }
    }
    
    m_internalCalibOffset = 0;
    m_cameraOffset = m_internalCalibOffset + scene.numInternalCalibs * NumUpdateParams::INTERNAL_CALIB;
    m_trackOffset = m_cameraOffset + scene.cameras.size() * NumUpdateParams::CAMERA;
    m_totalUpdateParams = m_trackOffset + scene.numTracks * NumUpdateParams::TRACK;
}

void BundleAdjustment::BAJacobiMatrix::multiply(float * __restrict dst, const float * __restrict src) const
{
    for (unsigned r = 0; r < m_rows.size(); r++) {
        float sumX = 0.0f;
        float sumY = 0.0f;
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            sumX += src[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] * 
                        m_rows[r].J_internalCalib(0, i);
            sumY += src[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] * 
                        m_rows[r].J_internalCalib(1, i);
        }
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            sumX += src[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] * 
                        m_rows[r].J_camera(0, i);
            sumY += src[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] * 
                        m_rows[r].J_camera(1, i);
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            sumX += src[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] * 
                        m_rows[r].J_track(0, i);
            sumY += src[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] * 
                        m_rows[r].J_track(1, i);
        }
        dst[r*2+0] = sumX;
        dst[r*2+1] = sumY;
    }
}

void BundleAdjustment::BAJacobiMatrix::transposedMultiply(float * __restrict dst, const float * __restrict src) const
{
    memset(dst, 0, sizeof(float) * m_totalUpdateParams);
    // This is super ugly...
    for (unsigned r = 0; r < m_rows.size(); r++) {
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            float elem = dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i];
            elem += src[r*2+0] * m_rows[r].J_internalCalib(0, i);
            elem += src[r*2+1] * m_rows[r].J_internalCalib(1, i);
            dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] = elem;
        }
        
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            float elem = dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i];
            elem += src[r*2+0] * m_rows[r].J_camera(0, i);
            elem += src[r*2+1] * m_rows[r].J_camera(1, i);
            dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            float elem = dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i];
            elem += src[r*2+0] * m_rows[r].J_track(0, i);
            elem += src[r*2+1] * m_rows[r].J_track(1, i);
            dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] = elem;
        }
    }
}

void BundleAdjustment::BAJacobiMatrix::computeDiagJtJ(float * __restrict dst) const
{
    memset(dst, 0, sizeof(float) * m_totalUpdateParams);
    // This is super ugly...
    for (unsigned r = 0; r < m_rows.size(); r++) {
        for (unsigned i = 0; i < NumUpdateParams::INTERNAL_CALIB; i++) {
            float elem = dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i];
            elem += m_rows[r].J_internalCalib(0, i) * m_rows[r].J_internalCalib(0, i);
            elem += m_rows[r].J_internalCalib(1, i) * m_rows[r].J_internalCalib(1, i);
            dst[m_internalCalibOffset + m_rows[r].internalCalibIdx * NumUpdateParams::INTERNAL_CALIB + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::CAMERA; i++) {
            float elem = dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i];
            elem += m_rows[r].J_camera(0, i) * m_rows[r].J_camera(0, i);
            elem += m_rows[r].J_camera(1, i) * m_rows[r].J_camera(1, i);
            dst[m_cameraOffset + m_rows[r].cameraIdx * NumUpdateParams::CAMERA + i] = elem;
        }
        for (unsigned i = 0; i < NumUpdateParams::TRACK; i++) {
            float elem = dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i];
            elem += m_rows[r].J_track(0, i) * m_rows[r].J_track(0, i);
            elem += m_rows[r].J_track(1, i) * m_rows[r].J_track(1, i);
            dst[m_trackOffset + m_rows[r].trackIdx * NumUpdateParams::TRACK + i] = elem;
        }
    }
}



BundleAdjustment::BAState::BAState(const Scene &scene) : m_scene(scene)
{
    m_tracks.resize(m_scene.numTracks);
    m_internalCalibs.resize(m_scene.numInternalCalibs);
    m_cameras.resize(m_scene.cameras.size());
}

OptimizationProblem::State* BundleAdjustment::BAState::clone() const
{
    return new BAState(m_scene);
}


BundleAdjustment::BundleAdjustment(Scene &scene) : m_scene(scene)
{
    m_numResiduals = 0;
    for (const auto &camera : m_scene.cameras)
        m_numResiduals += camera.keypoints.size()*2;
    
    m_numUpdateParameters = 
                m_scene.numInternalCalibs * NumUpdateParams::INTERNAL_CALIB +
                m_scene.cameras.size() * NumUpdateParams::CAMERA +
                m_scene.numTracks * NumUpdateParams::TRACK;
}

OptimizationProblem::JacobiMatrix* BundleAdjustment::createJacobiMatrix() const
{
    return new BAJacobiMatrix(m_scene);
}


void BundleAdjustment::downweightOutlierKeypoints(BAState &state)
{
    std::vector<float> residuals;
    residuals.resize(m_numResiduals);
    state.computeResiduals(residuals.data());
    
    std::vector<float> distances;
    distances.resize(m_numResiduals/2);
    
    unsigned residualIdx = 0;
    for (auto &c : m_scene.cameras) {
        for (auto &kp : c.keypoints) {
            distances[residualIdx/2] = 
                std::sqrt(residuals[residualIdx+0]*residuals[residualIdx+0] + 
                          residuals[residualIdx+1]*residuals[residualIdx+1]);
            residualIdx+=2;
        }
    }

    std::vector<float> sortedDistances = distances;
    std::sort(sortedDistances.begin(), sortedDistances.end());
    
    std::cout << "min, max, median distances (weighted): " << sortedDistances.front() << " " << sortedDistances.back() << " " << sortedDistances[sortedDistances.size()/2] << std::endl;
    
    float thresh = sortedDistances[sortedDistances.size() * 2 / 3] * 2.0f;
    
    residualIdx = 0;
    for (auto &c : m_scene.cameras)
        for (auto &kp : c.keypoints) 
            if (distances[residualIdx++] > thresh) 
                kp.weight *= 0.5f;
}


Scene buildScene(const std::vector<std::string> &imagesFilenames)
{
    const float threshold = 20.0f;
    
    struct Image {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        
        std::vector<std::vector<std::pair<unsigned, unsigned>>> matches;
    };
    
    std::vector<Image> allImages;
    allImages.resize(imagesFilenames.size());
    Ptr<ORB> orb = ORB::create();
    orb->setMaxFeatures(10000);
    for (unsigned i = 0; i < imagesFilenames.size(); i++) {
        std::cout << "Extracting keypoints from " << imagesFilenames[i] << std::endl;
        cv::Mat img = cv::imread(imagesFilenames[i].c_str());
        orb->detectAndCompute(img, cv::noArray(), allImages[i].keypoints, allImages[i].descriptors);
        allImages[i].matches.resize(allImages[i].keypoints.size());
    }
    
    Ptr<BFMatcher> matcher = BFMatcher::create(NORM_HAMMING);
    for (unsigned i = 0; i < allImages.size(); i++)
        for (unsigned j = i+1; j < allImages.size(); j++) {
            std::cout << "Matching " << imagesFilenames[i] << " against " << imagesFilenames[j] << std::endl;
            
            std::vector<std::vector<cv::DMatch>> matches;
            matcher->knnMatch(allImages[i].descriptors, allImages[j].descriptors, matches, 2);
            for (unsigned k = 0; k < matches.size(); ) {
                if (matches[k][0].distance > matches[k][1].distance * 0.75f) {
                    matches[k] = std::move(matches.back());
                    matches.pop_back();
                } else k++;
            }
            std::vector<cv::Vec3f> p1, p2;
            p1.resize(matches.size());
            p2.resize(matches.size());
            for (unsigned k = 0; k < matches.size(); k++) {
                p1[k] = cv::Vec3f(allImages[i].keypoints[matches[k][0].queryIdx].pt.x,
                                  allImages[i].keypoints[matches[k][0].queryIdx].pt.y,
                                  1.0f);
                p2[k] = cv::Vec3f(allImages[j].keypoints[matches[k][0].trainIdx].pt.x,
                                  allImages[j].keypoints[matches[k][0].trainIdx].pt.y,
                                  1.0f);
            }
            std::cout << "RANSACing " << imagesFilenames[i] << " against " << imagesFilenames[j] << std::endl;
            
            cv::Matx33f F = estimateFundamentalRANSAC(p1, p2, 1000, threshold);
            
            std::vector<std::pair<unsigned, unsigned>> inlierMatches;
            for (unsigned k = 0; k < matches.size(); k++) 
                if (getError(p1[k], p2[k], F) < threshold) 
                    inlierMatches.push_back({
                        matches[k][0].queryIdx,
                        matches[k][0].trainIdx
                    });
            const unsigned minMatches = 400;
                
            std::cout << "Found " << inlierMatches.size() << " valid matches!" << std::endl;
            if (inlierMatches.size() >= minMatches)
                for (const auto p : inlierMatches) {
                    allImages[i].matches[p.first].push_back({j, p.second});
                    allImages[j].matches[p.second].push_back({i, p.first});
                }
        }
    
    
    Scene scene;
    scene.numInternalCalibs = 1;
    scene.cameras.resize(imagesFilenames.size());
    for (auto &c : scene.cameras)
        c.internalCalibIdx = 0;
    scene.numTracks = 0;
    
    std::cout << "Finding tracks " << std::endl;
    {
        std::set<std::pair<unsigned, unsigned>> handledKeypoints;
        std::set<unsigned> imagesSpanned;
        std::vector<std::pair<unsigned, unsigned>> kpStack;
        std::vector<std::pair<unsigned, unsigned>> kpList;
        for (unsigned i = 0; i < allImages.size(); i++) {
            for (unsigned kp = 0; kp < allImages[i].keypoints.size(); kp++) {
                if (allImages[i].matches[kp].empty()) continue;
                if (handledKeypoints.find({i, kp}) != handledKeypoints.end()) continue;
                
                bool valid = true;
                
                kpStack.push_back({i, kp});
                while (!kpStack.empty()) {
                    auto kp = kpStack.back();
                    kpStack.pop_back();
                    
                    
                    if (imagesSpanned.find(kp.first) != imagesSpanned.end()) // appearing twice in one image -> invalid
                        valid = false;
                    
                    handledKeypoints.insert(kp);
                    kpList.push_back(kp);
                    imagesSpanned.insert(kp.first);
                    
                    for (const auto &matchedKp : allImages[kp.first].matches[kp.second])
                        if (handledKeypoints.find(matchedKp) == handledKeypoints.end()) 
                            kpStack.push_back(matchedKp);
                }
                
                if (valid) {
                    //std::cout << "Forming track from group of " << kpList.size() << " keypoints over " << imagesSpanned.size() << " images" << std::endl;
                    
                    for (const auto &kp : kpList) {
                        cv::Vec2f pixelPosition;
                        pixelPosition(0) = allImages[kp.first].keypoints[kp.second].pt.x;
                        pixelPosition(1) = allImages[kp.first].keypoints[kp.second].pt.y;
                        
                        unsigned trackIdx = scene.numTracks;
                        
                        scene.cameras[kp.first].keypoints.push_back({
                            pixelPosition,
                            trackIdx,
                            1.0f
                        });
                    }
                    
                    scene.numTracks++;
                } else {
                    //std::cout << "Dropping invalid group of " << kpList.size() << " keypoints over " << imagesSpanned.size() << " images" << std::endl;
                }
                kpList.clear();
                imagesSpanned.clear();
            }
        }
        std::cout << "Formed " << scene.numTracks << " tracks" << std::endl;
    }
    
    for (auto &c : scene.cameras)
        if (c.keypoints.size() < 100)
            std::cout << "Warning: One camera is connected with only " << c.keypoints.size() << " keypoints, this might be too unstable!" << std::endl;

    return scene;
}

void produceInitialState(const Scene &scene, const cv::Matx33f &initialInternalCalib, BundleAdjustment::BAState &state)
{
    const float threshold = 20.0f;
    
    state.m_internalCalibs[0].K = initialInternalCalib;
    
    std::set<unsigned> triangulatedPoints;
    
    const unsigned image1 = 0;
    const unsigned image2 = 1;
    // Find stereo pose of first two images
    {
        
        std::map<unsigned, cv::Vec2f> track2keypoint;
        for (const auto &kp : scene.cameras[image1].keypoints)
            track2keypoint[kp.trackIdx] = kp.location;
        
        std::vector<std::pair<cv::Vec2f, cv::Vec2f>> matches;
        std::vector<unsigned> matches2track;
        for (const auto &kp : scene.cameras[image2].keypoints) {
            auto it = track2keypoint.find(kp.trackIdx);
            if (it != track2keypoint.end()) {
                matches.push_back({it->second, kp.location});
                matches2track.push_back(kp.trackIdx);
            }
        }
        
        std::cout << "Initial pair has " << matches.size() << " matches" << std::endl;
        
        std::vector<cv::Vec3f> p1;
        p1.reserve(matches.size());
        std::vector<cv::Vec3f> p2;
        p2.reserve(matches.size());
        for (unsigned i = 0; i < matches.size(); i++) {
            p1.push_back(cv::Vec3f(matches[i].first(0), matches[i].first(1), 1.0f));
            p2.push_back(cv::Vec3f(matches[i].second(0), matches[i].second(1), 1.0f));
        }
        
        const cv::Matx33f &K = initialInternalCalib;
        state.m_cameras[image1].H = cv::Matx44f::eye();
        state.m_cameras[image2].H = computeCameraPose(K, p1, p2);
            
        std::vector<cv::Vec4f> X = linearTriangulation(K * cv::Matx34f::eye(), K * cv::Matx34f::eye() * state.m_cameras[image2].H, p1, p2);
        for (unsigned i = 0; i < X.size(); i++) {
            cv::Vec4f t = X[i];
            t /= std::sqrt(t.dot(t));
            state.m_tracks[matches2track[i]].location = t;
            triangulatedPoints.insert(matches2track[i]);
        }
    }
    

    for (unsigned c = 0; c < scene.cameras.size(); c++) {
        if (c == image1) continue;
        if (c == image2) continue;
        
        std::vector<KeyPoint> triangulatedKeypoints;
        for (const auto &kp : scene.cameras[c].keypoints) 
            if (triangulatedPoints.find(kp.trackIdx) != triangulatedPoints.end()) 
                triangulatedKeypoints.push_back(kp);

        if (triangulatedKeypoints.size() < 100)
            std::cout << "Warning: Camera " << c << " is only estimated from " << triangulatedKeypoints.size() << " keypoints" << std::endl;
        
        std::vector<cv::Vec3f> points2D;
        points2D.resize(triangulatedKeypoints.size());
        std::vector<cv::Vec4f> points3D;
        points3D.resize(triangulatedKeypoints.size());
        
        for (unsigned i = 0; i < triangulatedKeypoints.size(); i++) {
            points2D[i] = cv::Vec3f(
                        triangulatedKeypoints[i].location(0),
                        triangulatedKeypoints[i].location(1),
                        1.0f);
            points3D[i] = state.m_tracks[triangulatedKeypoints[i].trackIdx].location;
        }
        
        std::cout << "Estimating camera " << c << " from " << triangulatedKeypoints.size() << " keypoints" << std::endl;
        //cv::Mat P = calibrate(points2D, points3D);
        cv::Matx34f P = estimateProjectionRANSAC(points2D, points3D, 1000, threshold);
        cv::Matx33f K, R;
        ProjectionMatrixInterpretation info;
        interprete(P, K, R, info);
        
        state.m_cameras[c].H = cv::Matx44f::eye();
        for (unsigned i = 0; i < 3; i++)
            for (unsigned j = 0; j < 3; j++)
                state.m_cameras[c].H(i, j) = R(i, j);
            
        state.m_cameras[c].H = state.m_cameras[c].H * translationMatrix(-info.cameraLocation[0], -info.cameraLocation[1], -info.cameraLocation[2]);
    }
    // Triangulate remaining points
    for (unsigned c = 0; c < scene.cameras.size(); c++) {
        
        cv::Matx34f P1 = state.m_internalCalibs[scene.cameras[c].internalCalibIdx].K * cv::Matx34f::eye() * state.m_cameras[c].H;
            
        for (unsigned otherC = 0; otherC < c; otherC++) {
            cv::Matx34f P2 = state.m_internalCalibs[scene.cameras[otherC].internalCalibIdx].K * cv::Matx34f::eye() * state.m_cameras[otherC].H;
            for (const auto &kp : scene.cameras[c].keypoints) {
                if (triangulatedPoints.find(kp.trackIdx) != triangulatedPoints.end()) continue;
                
                for (const auto &otherKp : scene.cameras[otherC].keypoints) {
                    if (kp.trackIdx == otherKp.trackIdx) {
                        cv::Vec4f X = linearTriangulation(
                            P1, P2,
                            cv::Vec3f(kp.location(0), kp.location(1), 1.0f),
                            cv::Vec3f(otherKp.location(0), otherKp.location(1), 1.0f)
                        );
                        
                        X /= std::sqrt(X.dot(X));
                        state.m_tracks[kp.trackIdx].location = X;
                        
                        triangulatedPoints.insert(kp.trackIdx);
                    }
                }
            }
        }
    }
    if (triangulatedPoints.size() != state.m_tracks.size())
        std::cout << "Warning: Some tracks were not triangulated. This should not happen!" << std::endl;
}


}
