#pragma once

#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>
#include<vector>
#include<math.h>
#include<bitset>
#include<sstream>

/**
subpixSampleSafe
Function: Calculate color of subpixel values from grayscale iamge

@param img, grayscale image of current frame
@param sub_pixel, a point representing a location in subpixel accuracy
*/
int subpixSampleSafe(const cv::Mat& img, const cv::Point2f& sub_pixel);

/**
preciseEdgePoint
Function: Find the most precise location of a edge point from a stripe of subpixel values

@param img, stripe of subpixel values
@param edge_point, the original delimiting edge point calculated
@param stripeVecY, direction of Y axis of stripe
@param stripeLength, length of stripe

*/
cv::Point2f preciseEdgePoint(const cv::Mat& img,
	const cv::Point2f& edge_point,
	const cv::Point2f& stipeVecY,
	int stripeLength);

/**
computeCorners
Function: More precisely calculate four corners of candidate from lines fitted through edge points

@param lines, a vector of sets of line parameters (vx,vy,x0,y0), point (x0,y0) and direction v
@param corners, an array of corner points (to be populated within function)
*/
void computeCorners(const std::vector<cv::Vec4f>& lines, cv::Point2f corners[4]);

/**
findIntersection
Function: Find the intersection betwen two lines using Cramer's rule

@param L1, a line formatted as (vx,vy,x0,y0)
@param L2, a line formatted as (vx,vy,x0,y0)
*/
cv::Point2f findIntersection(const cv::Vec4f& L1, const cv::Vec4f& L2);

/**
getID
Function: calculate hexadecimal id of interior image for
		  four different rotations

@param markerImg, 6x6 cv::Mat which is homography transformation of candidate
@param num, highest intensity value
@param numRotations, how many rotations required to get smallest ID
*/
std::string getID(const cv::Mat& markerImg, int num, int& numRotations);

/**
calculateID
Function: Calculate hexadeimcal ID of interior image

@param subImg, a 4x4 sub matrix of 6x6 candidate
@param num, the number 255 (so that full intensity maps to 1)
*/
std::string calculateID(const cv::Mat& subImg, int num);

/**
checkBorder
Function: Determine if markers (interior sub-image) have a black border,
		  returns true for all black or else returns true

@param markerImg, 6x6 cv::Mat which is homography transformation of candidate
*/
bool checkBorder(const cv::Mat& markerImg);

/**
shiftPoints
Function: Do a circular rotation of points in array by specified nuber of times

@param oldCorners, orginal non-cycled corners
@param newCorners, corners after being shifted by numRotations
@param numRotations, the number of array right shifts to perform
*/
void shiftPoints(cv::Point2f(&oldCorners)[4], cv::Point2f newCorners[4], int numRotations);

/**
displayPose
Function: print pose to console as 4x4 matrix

@param pose, a 4x4 homogeneous transformation matrix
*/
void displayPose(float pose[][4]);


/**
showCoordinateFrame
Function: Draw the frame axes cooresponding to 3D pose

@param img, current frame to be drawn on 
@param pose, a 4x4 Homogeneous transformation matrix
@param K, the camera intrinsics matrix 
@param dist, camera's distortion coefficients vector, such as: (k1, k2, p1, p2, k3)
*/
void showCoordinateFrame(cv::Mat img, float pose[][4], const cv::Mat& K, const std::vector<float>& dist); 

