#include "Tracking.h"

/**
subpixSampleSafe
Function: Calculate color of subpixel values from grayscale iamge

@param img, grayscale image of current frame
@param sub_pixel, a point representing a location in subpixel accuracy
*/
int subpixSampleSafe(const cv::Mat& img, const cv::Point2f& sub_pixel) {

	int I;

	// Needed pixel length variables 
	int fx = std::floor(sub_pixel.x);
	int fy = std::floor(sub_pixel.y);
	int px = 1 - fx;
	int py = 1 - fy;

	// check fx 
	if (!((fx >= 0) && (fx <= img.cols - 2))) {

		if (fx > img.cols - 2) {
			fx = img.cols - 2;
		}
		else if (fx < 0) {
			fx = 0;
		}
	}

	// check fy
	if (!((fy >= 0) && (fy <= img.rows - 2))) {

		if (fy > img.rows - 2) {
			fy = img.rows - 2;
		}
		else if (fy < 0) {
			fy = 0;
		}
	}

	// Calculate intensity from pixel intensities 
	I = (1 - py)*((1 - px)*img.at<uchar>(fy, fx) + px * img.at<uchar>(fy, fx + 1)) +
		py * ((1 - px)*img.at<uchar>(fy + 1, fx) + px * img.at<uchar>(fy + 1, fx + 1));

	return I;
}

/**
preciseEdgePoint
Function: Find the most precise location of a edge point from a stripe of subpixel values

@param img, stripe of subpixel values
@param edge_point, the original delimiting edge point calculated
@param stripeVecY, direction of Y axis of stripe
@param stripeLength, length of stripe

*/
cv::Point2f preciseEdgePoint(
	const cv::Mat& stripe,
	const cv::Point2f& edge_point,
	const cv::Point2f& stripeVecY,
	int stripeLength) {

	// Find the largest value location 
	int largest = stripe.at<uchar>(0, 0);
	cv::Point large(0, 0);
	for (int i = 0; i < stripe.rows; ++i) {
		for (int j = 0; j < stripe.cols; ++j) {
			if (stripe.at<uchar>(i, j) > largest) {
				largest = stripe.at<uchar>(i, j);
				large = cv::Point(j, i);
			}
		}
	}

	// Central position of Stripe matrix refers to current edge point 
	int center_row = (int)(stripeLength) / 2;
	int center_col = 1;
	int row_disp = large.y - center_row;
	int col_disp = large.x - center_col;
	int difference = row_disp;

	return cv::Point2f(edge_point.x + col_disp * stripeVecY.x,
		edge_point.y + row_disp * stripeVecY.y);
}

/**
computeCorners
Function: More precisely calculate four corners of candidate from lines fitted through edge points

@param lines, a vector of sets of line parameters (vx,vy,x0,y0), point (x0,y0) and direction v
@param corners, an array of corner points (to be populated within function)
*/
void computeCorners(const std::vector<cv::Vec4f>& lines, cv::Point2f corners[4]) {

	for (int i = 0; i < 4; ++i) {
		corners[i] = findIntersection(lines[i], lines[(i + 3) % 4]);
	}
}

/**
findIntersection
Function: Find the intersection betwen two lines using Cramer's rule

@param L1, a line formatted as (vx,vy,x0,y0)
@param L2, a line formatted as (vx,vy,x0,y0)
*/
cv::Point2f findIntersection(const cv::Vec4f& L1, const cv::Vec4f& L2) {

	// Solve 2D Linear System with Cramer's Rule
	float e = -L1[1] * L1[2] + L1[0] * L1[3];
	float f = -L2[1] * L2[2] + L2[0] * L2[3];
	float detA = -L1[1] * L2[0] - (L1[0] * (-L2[1])); // ad - bc
	float detOne = e * L2[0] - L1[0] * f;             // ed - bf
	float detTwo = -L1[1] * f - e * (-L2[1]);         // af - ec

	return cv::Point2f(detOne / detA, detTwo / detA);
}

/**
getID
Function: calculate hexadecimal id of interior image for
		  four different rotations

@param markerImg, 6x6 cv::Mat which is homography transformation of candidate
@param num, highest intensity value
@param numRotations, how many rotations required to get smallest ID
*/
std::string getID(const cv::Mat& markerImg, int num, int& numRotations) {

	cv::Mat ROI = markerImg(cv::Rect(1, 1, 4, 4));
	cv::Mat ROI_dst(cv::Size(6, 6), CV_8UC1);
	std::string hexIDs[4];

	// orginal 
	hexIDs[0] = calculateID(ROI, num);

	// rotate by 90 degrees
	cv::rotate(ROI, ROI_dst, cv::ROTATE_90_CLOCKWISE);
	hexIDs[1] = calculateID(ROI_dst, num);

	// rotate by 180 degrees
	cv::rotate(ROI, ROI_dst, cv::ROTATE_180);
	hexIDs[2] = calculateID(ROI_dst, num);

	// rotate by 270 degrees
	cv::rotate(ROI, ROI_dst, cv::ROTATE_90_COUNTERCLOCKWISE);
	hexIDs[3] = calculateID(ROI_dst, num);

	// Smallest value and index 
	std::string* small = std::min_element(hexIDs, hexIDs + 4);
	ptrdiff_t smallIndex = small - &hexIDs[0];
	numRotations = smallIndex;

	return *small;
}

/**
calculateID
Function: Calculate hexadeimcal ID of interior image

@param subImg, a 4x4 sub matrix of 6x6 candidate
@param num, the number 255 (so that full intensity maps to 1)
*/
std::string calculateID(const cv::Mat& subImg, int num) {

	// Holds binary strings 
	std::string values[4];

	// Holds hex string 
	std::string hexValue;

	// Turn binary matrix with uchar values of 255 or 0 into 
	// binary matrix with values of zero or one 
	// But also invert the positive and zero values 
	cv::Mat subImg_new = (~subImg) / num;

	std::stringstream result; // store ID 

	// Loop through sub matrix and add on binary digits 
	int temporary = 0;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			temporary = (int)subImg_new.at<uchar>(i, j);
			values[i] += std::to_string(temporary);
		}
		std::bitset<4> set(values[i]);
		result << std::hex << std::uppercase << set.to_ulong();
		//std::cout << result.str();
	}

	return result.str();
}

/**
checkBorder
Function: Determine if markers (interior sub-image) have a black border,
		  returns true for all black or else returns true

@param markerImg, 6x6 cv::Mat which is homography transformation of candidate
*/
bool checkBorder(const cv::Mat& markerImg) {

	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 6; ++j) {
			if (i == 0 || j == 0 || i == 5 || j == 5) {
				if (!(markerImg.at<uchar>(i, j) == 0)) {
					return false;
				}
			}
		}
	}
	return true;
}

/**
shiftPoints
Function: Do a circular rotation of points in array by specified nuber of times

@param oldCorners, orginal non-cycled corners
@param newCorners, corners after being shifted by numRotations
@param numRotations, the number of array right shifts to perform
*/
void shiftPoints(cv::Point2f(&oldCorners)[4], cv::Point2f newCorners[4], int numRotations) {

	for (int i = 0; i < 4; ++i) {
		newCorners[(i + numRotations) % 4] = oldCorners[i];
	}
}

/**
displayPose
Function: print pose to console as 4x4 matrix

@param pose, a 4x4 homogeneous transformation matrix
*/
void displayPose(float pose[][4]) {

	std::cout << "[";
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			if (pose[i][j] == 0 || pose[i][j] == 1) { std::cout << std::setw(7); }
			if (i == 3 && j == 3) { std::cout << pose[i][j];  continue; }
			std::cout << pose[i][j] << ", ";
		}if (i != 3) { std::cout << std::endl; }
	}
	std::cout << "]" << std::endl;
}

/**
showCoordinateFrame
Function: Draw the frame axes cooresponding to 3D pose

@param img, current frame to be drawn on
@param pose, a 4x4 Homogeneous transformation matrix
@param K, the camera intrinsics matrix
@param dist, camera's distortion coefficients vector, such as: (k1, k2, p1, p2, k3)
*/
void showCoordinateFrame(cv::Mat img, float pose[][4], const cv::Mat& K, const std::vector<float>& dist) {

	// Rotation Matrix and Vector 
	float rot_vals[9] = { pose[0][0],pose[0][1],pose[0][2],pose[1][0],pose[1][1],
		pose[1][2],pose[2][0],pose[2][1],pose[2][2] };
	cv::Mat rot_mtx = cv::Mat(3, 3, CV_32F, rot_vals);
	std::vector<float> rot_vec(3);
	cv::Rodrigues(rot_mtx, rot_vec);

	// Translation Vector
	std::vector<float> t_vec = { (float)1.0*pose[0][3],(float)1.0*pose[1][3],(float)1.0*pose[2][3] };

	/* Draw Frame Axes --------------------------------------------------------------------------- */

	// 3D points respresentations of unit vectors 
	std::vector<cv::Point3f> points;
	points.reserve(4);
	points.push_back(cv::Point3f(0.01, 0.0, 0.0));
	points.push_back(cv::Point3f(0.0, 0.01, 0.0));
	points.push_back(cv::Point3f(0.0, 0.0, 0.01));
	points.push_back(cv::Point3f(0.0, 0.0, 0.0));

	// Project points 3D --> 2D 
	//std::vector<float> dist_ = { 0.0,0.0,0.0,0.0 }; // test without distortion correction
	std::vector<cv::Point2f> axisPoints;
	cv::projectPoints(points, rot_vec, t_vec, K, dist, axisPoints);

	// Draw lines using RGB convention
	cv::arrowedLine(img, axisPoints[3], axisPoints[0], cv::Scalar(0, 0, 255), 3, CV_AA);
	cv::arrowedLine(img, axisPoints[3], axisPoints[1], cv::Scalar(0, 255,0), 3, CV_AA);
	cv::arrowedLine(img, axisPoints[3], axisPoints[2], cv::Scalar(255, 0, 0), 3, CV_AA);
}