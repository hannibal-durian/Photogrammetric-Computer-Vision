//============================================================================
// Name        : Pcv4.cpp
// Author      : Ronny Haensch, Andreas Ley
// Version     : 2.0
// Copyright   : -
// Description : Estimation of Fundamental Matrix
//============================================================================

#include "Pcv4.h"

#include <random>
#include <opencv2/features2d.hpp>


using namespace cv;
using namespace std;


namespace pcv4 {

    /**
 * @brief Applies a 2D transformation to an array of points or lines
 * @param H Matrix representing the transformation
 * @param geomObjects Array of input objects, each in homogeneous coordinates
 * @param type The type of the geometric objects, point or line. All are the same type.
 * @returns Array of transformed objects.
 */
std::vector<cv::Vec3f> applyH_2D(const std::vector<cv::Vec3f>& geomObjects, const cv::Matx33f &H, GeometryType type)
{
    // TO DO !!!
	std::vector<cv::Vec3f> result;
	switch (type) {
	case GEOM_TYPE_POINT: {
		// TO DO !!!
		for (int i = 0; i < geomObjects.size(); i++)
		{
			result.push_back(H * geomObjects[i]);
		}
	} break;
	case GEOM_TYPE_LINE: {
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
    //return {};
}


/**
 * @brief Get the conditioning matrix of given points
 * @param p The points as matrix
 * @returns The condition matrix
 */
cv::Matx33f getCondition2D(const std::vector<cv::Vec3f>& points2D)
{
    // TO DO !!!
	float num_x = 0;
	float num_y = 0;
	for (int i = 0; i < points2D.size(); ++i)
	{
		num_x += points2D[i][0];
		num_y += points2D[i][1];
	}
	float t_x = num_x / points2D.size();
	float t_y = num_y / points2D.size();
	std::vector<cv::Vec3f> new_points = points2D;
	for (int i = 0; i < points2D.size(); ++i)
	{
		new_points[i][0] = points2D[i][0] - t_x;
		new_points[i][1] = points2D[i][1] - t_y;
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
 * @brief Define the design matrix as needed to compute fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @returns The design matrix to be computed
 */
cv::Mat_<float> getDesignMatrix_fundamental(const std::vector<cv::Vec3f>& p1_conditioned, const std::vector<cv::Vec3f>& p2_conditioned)
{
    // TO DO !!!
	int n = p1_conditioned.size();
	cv::Mat A = cv::Mat_<float>(n, 9);
	for (int i = 0; i < n; i++)
	{
		A.at<float>(i, 0) = p1_conditioned[i][0] * p2_conditioned[i][0];
		A.at<float>(i, 1) = p1_conditioned[i][1] * p2_conditioned[i][0];
		A.at<float>(i, 2) = p1_conditioned[i][2] * p2_conditioned[i][0];

		A.at<float>(i, 3) = p1_conditioned[i][0] * p2_conditioned[i][1];
		A.at<float>(i, 4) = p1_conditioned[i][1] * p2_conditioned[i][1];
		A.at<float>(i, 5) = p1_conditioned[i][2] * p2_conditioned[i][1];

		A.at<float>(i, 6) = p1_conditioned[i][0] * p2_conditioned[i][2];
		A.at<float>(i, 7) = p1_conditioned[i][1] * p2_conditioned[i][2];
		A.at<float>(i, 8) = p1_conditioned[i][2] * p2_conditioned[i][2];

	}
	return A;
    //return cv::Mat_<float>();
}


/**
 * @brief Solve homogeneous equation system by usage of SVD
 * @param A The design matrix
 * @returns The estimated fundamental matrix
 */
cv::Matx33f solve_dlt_fundamental(const cv::Mat_<float>& A)
{
    // TO DO !!!
    cv::Mat A_new;
    if (A.rows==8)
    {
        cv::copyMakeBorder(A, A_new, 0, 1, 0, 0, cv::BORDER_CONSTANT, 0);
        //A_new = cv::Mat::zeros(9, 9,A.type());
        //for (int a=0; a<A.rows; a++)
        //{
         //   for (int b=0; b<A.cols; b++)
          //  {
            //    A_new.at<float>(a,b) = A.at<float>(a,b);
          //  }
        //}
    }
    else
    {
        A_new = A.clone();
    }
	cv::SVD svd(A_new, cv::SVD::FULL_UV);
	float* h= svd.vt.ptr<float>(8);
	return cv::Matx33f(h[0],h[1],h[2],h[3],h[4],h[5],h[6],h[7],h[8]);
	//cv::Matx33f F;
	//F(0, 0) = svd.vt.at<float>(8, 0);
	//F(0, 1) = svd.vt.at<float>(8, 1);
	//F(0, 2) = svd.vt.at<float>(8, 2);
	//F(1, 0) = svd.vt.at<float>(8, 3);
	//F(1, 1) = svd.vt.at<float>(8, 4);
	//F(1, 2) = svd.vt.at<float>(8, 5);
	//F(2, 0) = svd.vt.at<float>(8, 6);
	//F(2, 1) = svd.vt.at<float>(8, 7);
	//F(2, 2) = svd.vt.at<float>(8, 8);

	//cv::resize(svd.vt.at<float>(8,),F,Size(3,3))
	//return F;
    //return cv::Matx33f::zeros();
}


/**
 * @brief Enforce rank of 2 on fundamental matrix
 * @param F The matrix to be changed
 * @return The modified fundamental matrix
 */
cv::Matx33f forceSingularity(const cv::Matx33f& F)
{
    // TO DO !!!
	cv::SVD svd(F, cv::SVD::FULL_UV);
	svd.w.at<float>(2) = 0;
	cv::Mat F_new = svd.u * cv::Mat::diag(svd.w) * svd.vt;
	return F_new;
    //return F;
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
    // TO DO !!!
	cv::Mat T2_trans;
	cv::transpose(T2, T2_trans);
	cv::Mat F_decondition = T2_trans * F * T1;
	return F_decondition;
    //return F;
}


/**
 * @brief Compute the fundamental matrix
 * @param p1 first set of points
 * @param p2 second set of points
 * @return The estimated fundamental matrix
 */
cv::Matx33f getFundamentalMatrix(const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2)
{
    // TO DO !!!
	cv::Matx33f T1 = getCondition2D(p1);
	cv::Matx33f T2 = getCondition2D(p2);
	std::vector<cv::Vec3f> conditioned_1 = applyH_2D(p1, T1, GEOM_TYPE_POINT);
	std::vector<cv::Vec3f> conditioned_2 = applyH_2D(p2, T2, GEOM_TYPE_POINT);
	cv::Mat_<float> A = getDesignMatrix_fundamental(conditioned_1, conditioned_2);
	cv::Matx33f F_ = solve_dlt_fundamental(A);
	cv::Matx33f F_rank2 = forceSingularity(F_);
	cv::Matx33f F_decondition = decondition_fundamental(T1, T2, F_rank2);
	return F_decondition;
    //return cv::Matx33f::eye();
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
    // TO DO !!!
	float d;
	cv::Matx33f F_trans;
	cv::transpose(F, F_trans);
	cv::Vec3f l2 = F * p1;
	cv::Vec3f l1 = F_trans * p2;
	float a_l2 = l2[0];
	float b_l2 = l2[1];
	float a_l1 = l1[0];
	float b_l1 = l1[1];
	//cv::Vec3f p2_T;
	//cv::transpose(p2, p2_T);
	//float m = p2_T * F * p1;
	float m = p2[0] * l2[0] + p2[1] * l2[1] + p2[2] * l2[2];
	d = pow(m, 2) / (pow(a_l2, 2) + pow(b_l2, 2) + pow(a_l1, 2) + pow(b_l1, 2));
	return d;
    //return 0.0f;
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
    // TO DO !!!
	int n = p1.size();
	float sum = 0;
	for (int i = 0; i < n; i++)
	{
		sum += getError(p1[i], p2[i], F);
	}
	float err = sum / n;
	return err;
    //return 0.0f;
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
    // TO DO !!!
	int n = p1.size();
	float counting = 0;
	for (int i = 0; i < n; i++)
	{
		if (getError(p1[i], p2[i], F) < threshold)
		{
			counting += 1;
		}
	}
	return counting;
    //return 0;
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
    const unsigned subsetSize = 8;

    std::mt19937 rng;
    std::uniform_int_distribution<unsigned> uniformDist(0, p1.size()-1);
    // Draw a random point index with unsigned index = uniformDist(rng);

    // TO DO !!!
	int counting = 0;
	cv::Matx33f  F = cv::Matx33f::eye();
	for (int j = 0; j < numIterations; j++)
	{
		std::vector<cv::Vec3f> p1_new;
		std::vector<cv::Vec3f> p2_new;
		for (size_t i = 0; i < subsetSize; ++i)
		{
			unsigned index = uniformDist(rng);
			p1_new.push_back(p1[index]);
			p2_new.push_back(p2[index]);
		}

		cv::Matx33f F_new = getFundamentalMatrix(p1_new, p2_new);
		int count_new = countInliers(p1, p2, F_new, threshold);
		if (counting < count_new)
		{
			counting = count_new;
			F = F_new;
		}
	}
	return F;
	//return cv::Matx33f::eye();
}




/**
 * @brief Draw points and corresponding epipolar lines into both images
 * @param img1 Structure containing first image
 * @param img2 Structure containing second image
 * @param p1 First point set (points in first image)
 * @param p2 First point set (points in second image)
 * @param F Fundamental matrix (mapping from point in img1 to lines in img2)
 */
void visualize(const cv::Mat& img1, const cv::Mat& img2, const std::vector<cv::Vec3f>& p1, const std::vector<cv::Vec3f>& p2, const cv::Matx33f& F)
{
    // make a copy to not draw into the original images and destroy them
    cv::Mat img1_copy = img1.clone();
    cv::Mat img2_copy = img2.clone();

    // TO DO !!!
    // Compute epilines for both images and draw them with drawEpiLine() into img1_copy and img2_copy respectively
    // Use cv::circle(image, cv::Point2f(x, y), 2, cv::Scalar(0, 255, 0), 2); to draw the points.
	int n = p1.size();
	for (int i = 0; i < n; i++)
	{
		cv::Matx33f F_trans;
		cv::transpose(F, F_trans);
		cv::Vec3f l2 = F * p1[i];
		cv::Vec3f l1 = F_trans * p2[i];
		cv::circle(img1_copy, cv::Point2f(p1[i][0], p1[i][1]), 2, cv::Scalar(0, 255, 0), 2);
		cv::circle(img2_copy, cv::Point2f(p2[i][0], p2[i][1]), 2, cv::Scalar(0, 255, 0), 2);
		drawEpiLine(img1_copy, l1[0], l1[1], l1[2]);
		drawEpiLine(img2_copy, l2[0], l2[1], l2[2]);
	}

    // show images
    cv::imshow("Epilines img1", img1_copy);
    cv::imshow("Epilines img2", img2_copy);
    cv::imwrite("fstImage.png", img1_copy);
    cv::imwrite("sndImage.png", img2_copy);

    cv::waitKey(0);
}



/**
 * @brief Filters the raw matches
 * @details Applies cross consistency check and ratio test (ratio of 0.75) and returns the point pairs that pass both.
 * @param rawOrbMatches Structure containing keypoints and raw matches obtained from comparing feature descriptors (see Helper.h)
 * @param p1 Points within the first image (returned in the array by this method)
 * @param p2 Points within the second image (returned in the array by this method)
 */
void filterMatches(const RawOrbMatches &rawOrbMatches, std::vector<cv::Vec3f>& p1, std::vector<cv::Vec3f>& p2)
{

/******* Small std::map cheat sheet ************************************

// This std::map stores pairs of ints and floats (key value pairs). Each float (value) can quickly be looked up with it's corresponding int (key).
std::map<int, float> exampleMap;

// Looking up an element:
int key = 5;
auto it = exampleMap.find(key);
if (it == exampleMap.end()) {
    // no entry with key 5 in the map
} else {
    float value = it->second;
    // do s.th. with the value
}

// Iteration over all elements:
for (const auto &pair : exampleMap) {
    int key = pair.first;
    float value = pair.second;
}

**************************************************************************/

    p1.clear();
    p2.clear();

    const float ratio = 0.75f;

    for (const auto &pair : rawOrbMatches.matches_1_2) {

        // TO DO !!!
        // Skip those pairs that don't fulfill the ratio test or cross consistency check
        if (pair.second.closestDistance/pair.second.secondClosestDistance < ratio)
        {
            auto cloestImage2 = rawOrbMatches.matches_2_1.find(pair.second.closest)->second;
            if (cloestImage2.closest == pair.first)
            {
                p1.push_back(rawOrbMatches.keypoints1[pair.first]);
                p2.push_back(rawOrbMatches.keypoints2[pair.second.closest]);
            }
        }


        //p1.push_back(rawOrbMatches.keypoints1[pair.first]);
        //p2.push_back(rawOrbMatches.keypoints2[pair.second.closest]);
    }
}

/**
 * @brief Computes matches automatically.
 * @details Points will be in homogeneous coordinates.
 * @param img1 The first image
 * @param img2 The second image
 * @param p1 Points within the first image (returned in the array by this method)
 * @param p2 Points within the second image (returned in the array by this method)
 */
void getPointsAutomatic(const cv::Mat &img1, const cv::Mat &img2, std::vector<cv::Vec3f>& p1, std::vector<cv::Vec3f>& p2)
{
    // TO DO !!!
	const RawOrbMatches rawOrbMatches = extractRawOrbMatches(img1, img2);
	filterMatches(rawOrbMatches, p1, p2);
	//optional
	cv::Mat img1_copy = img1.clone();
	cv::Mat img2_copy = img2.clone();
	cv::Mat Image = drawMatches(img1_copy, img2_copy, p1, p2);
}


}
