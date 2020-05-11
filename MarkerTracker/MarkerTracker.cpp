
//#include<opencv2/opencv.hpp>
//#include<iostream>
//#include<string>
//#include<vector>
//#include<math.h>
#include "Tracking.h"
#include "PoseEstimation.h"

int main() {

	// Calibration results 
	float calib_values[9] = { 983.59084901, 0., 628.37970587, 0., 986.0985627, 305.17229519, 0., 0., 1. };
	cv::Mat K = cv::Mat(3, 3, CV_32F, calib_values);
	std::vector<float> dist = { 0.15888421, -0.44565814, -0.00143173,  0.00052475, 0.33254615 };

	// Constants 
	int threshold_value = 100;
	int max_BINARY_value = 255;

	cv::Mat frame; // video frame

	cv::VideoCapture cap; // video capture object
	int deviceID = 0; // Default camerax 
	int apiID = cv::CAP_ANY;  //  autodetect default API
	cap.open(deviceID, apiID); // open default camera using autodetected default api 

	/* Ensure that camera opened properly */
	if (!cap.isOpened()) {
		std::cerr << "ERROR, camera was not opened \n";
		return -1;
	}

	std::cout << "Begin grabbing frames... " << std::endl;
	std::cout << "Press any button to terminate " << std::endl;

	while (true) {

		// Wait to get new frame from camera and store in frame
		cap.read(frame);

		// Check for success 
		if (frame.empty()) {
			std::cerr << "ERROR, blank frame grabbed \n";
			break;
		}

		/* Begin Tracking Code */
		/*------------------------------------------------------------------------------------------*/

		// For debugging 
		// cv::setBreakOnError(true);

		cv::Mat frame_gray;
		cv::Mat frame_thresh;

		// Extracted Properties 
		int num_rows = frame.rows;
		int num_cols = frame.cols;

		// Variables for findings contour 
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;

		// Grayscale, then threshold 
		cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
		cv::threshold(frame_gray, frame_thresh, threshold_value, max_BINARY_value, CV_THRESH_BINARY);

		// Look for countours 
		cv::findContours(frame_thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		// Cycle through all contours, which are each vectors of points
		std::vector<std::vector<cv::Point>>::const_iterator contour = contours.begin();
		for (; contour != contours.end(); ++contour) {

			// Approximate polygon within error bounds based on size of the contour 
			std::vector<cv::Point> result;
			cv::approxPolyDP(*contour, result, cv::arcLength(*contour, true)*0.02, true);
			cv::Mat result_ = cv::Mat(result); // Store the resulting polgon approximation as a cv::Mat 

			// Find the upright bouding rectangle
			cv::Rect r = cv::boundingRect(result_);

			// Only go on if this is a four sided polygon, and of a good size 
			if (result.size() != 4) { continue; }
			if (r.area() < 80 || (r.width < 25 || r.height < 25)) { continue; }

			// Draw that polygon  
			const cv::Point *rect = (const cv::Point*) result_.data;
			int npts = result_.rows;
			cv::polylines(frame, &rect, &npts, 1, true, cv::Scalar(0, 0, 255), 2);

			// To store 
			std::vector<cv::Vec4f> lines;
			lines.resize(4);

			// Go over corner points 
			for (int i = 0; i < 4; ++i) {

				// Draw corners
				cv::circle(frame, rect[i], 2, cv::Scalar(0, 255, 0), 2);

				// Get the direction vector between this corner point and the next
				float dx = (rect[(i + 1) % 4].x - rect[i].x) / (float)7.0;
				float dy = (rect[(i + 1) % 4].y - rect[i].y) / (float)7.0;

				// Create stripe dimensions
				int stripeLength = (int)(0.8*sqrt(dx*dx + dy * dy));
				if (stripeLength < 5) { stripeLength = 5; }
				if (stripeLength % 2 != 0) { stripeLength++; }
				int nStop = (stripeLength - 1) / 2;
				int nStart = -nStop;
				cv::Size stripeSize;
				stripeSize.width = 3;
				stripeSize.height = stripeLength;

				// Direction vectors
				cv::Point2f stripeVecX;
				cv::Point2f stripeVecY;
				float diffLength = std::sqrt(dx*dx + dy * dy);
				stripeVecX.x = dx / diffLength;
				stripeVecX.y = dy / diffLength;
				stripeVecY.x = stripeVecX.y;
				stripeVecY.y = -stripeVecX.x;

				// Storage for more precise edge points 
				std::vector<cv::Point2f> betterPoints;
				betterPoints.reserve(6);

				// Mark Edges with seven spaces using six delimiters
				for (int j = 1; j < 7; ++j) {

					// Calculate and mark edge point 
					float px = rect[i].x + dx * j;
					float py = rect[i].y + dy * j;
					cv::circle(frame, cv::Point((int)px, (int)py), 2, cv::Scalar(255, 0, 0), 2);

					cv::Mat myStripe(stripeSize, CV_8UC1);

					// Strip Width
					for (int m = -1; m <= 1; ++m) {

						// Strip Length: This depends on the stripelength value calculated two loops up 
						for (int n = nStart; n <= nStop; ++n) {

							cv::Point2f subPixel;
							subPixel.x = px + (m * (stripeVecX.x + stripeVecY.x));
							subPixel.y = py + (n * (stripeVecX.y + stripeVecY.y));

							int pixel = subpixSampleSafe(frame_gray, subPixel);
							int w = m + 1;
							int h = n + (stripeLength >> 1);

							myStripe.at<uchar>(h, w) = (uchar)pixel;

						} // end loop over stripe length 
					} // end loop over stripe width 

					// Perform sobel operation 
					cv::Mat myStripe_prime(stripeSize, CV_8UC1);
					cv::Sobel(myStripe, myStripe_prime, CV_8UC1, 0, 1, 3);

					// Calculate and store more precise points
					cv::Point2f precise_point = preciseEdgePoint(myStripe_prime, cv::Point2f(px, py), stripeVecY, stripeLength);
					betterPoints.push_back(precise_point);

				} // end loop over six edge points

				// Fit line for this side 				
				cv::fitLine(betterPoints, lines[i], CV_DIST_L2, 0, 0.02, 0.01);

			} // end loop over four edges of shapes 

			// calculate corner points using the line parameters
			cv::Point2f corners[4];
			computeCorners(lines, corners);

			// Get matrix of perpspective transform  
			cv::Point2f dst_points[4];
			dst_points[0] = cv::Point2f(-0.5, -0.5);
			dst_points[1] = cv::Point2f(5.5, -0.5);
			dst_points[2] = cv::Point2f(5.5, 5.5);
			dst_points[3] = cv::Point2f(-0.5, 5.5);
			cv::Mat projMat = cv::getPerspectiveTransform(corners, dst_points);

			// Create marker image
			cv::Size markerSize(6, 6);
			cv::Mat markerImg(markerSize, CV_8UC1);
			cv::Mat markerImg_thresh(markerSize, CV_8UC1);
			cv::warpPerspective(frame_gray, markerImg, projMat, markerSize);
			cv::threshold(markerImg, markerImg_thresh, threshold_value, max_BINARY_value, CV_THRESH_BINARY);

			// Only identify candidates that could be markers
			if (!checkBorder(markerImg_thresh)) { continue; }

			// Find Marker ID, and number of rotations 
			int numRotations;
			std::string ret = getID(markerImg_thresh, 255, numRotations);

			// Shift corner points within array 
			cv::Point2f newCorners[4];
			shiftPoints(corners, newCorners, numRotations);

			// Adjust corner points for pose estimation 
			newCorners[0].x = -newCorners[0].x + num_cols/2;
			newCorners[1].x = -newCorners[1].x + num_cols/2;
			newCorners[2].x = -newCorners[2].x + num_cols/2;
			newCorners[3].x = -newCorners[3].x + num_cols/2;

			newCorners[0].y = -newCorners[0].y + num_rows/2;
			newCorners[1].y = -newCorners[1].y + num_rows/2;
			newCorners[2].y = -newCorners[2].y + num_rows/2;
			newCorners[3].y = -newCorners[3].y + num_rows/2;
			
			float markerLength = 4.5 / 100;    // compute marker side length (meters)

			// Estimate and print pose 
			float pose[4][4];
			estimateSquarePose(&pose[0][0], newCorners, markerLength);
			
			// Display Pose Estimate 
			//std::cout << "Pose Esimate: " << std::endl;
			//displayPose(pose);
			//std::cout << "-----------------------" << std::endl;

			// Draw coordinate axes 
			showCoordinateFrame(frame, pose, K, dist);

			// Display Marker ID 
			cv::putText(frame,
				ret,
				cv::Point((int)corners[0].x, (int)corners[0].y), // Coordinates
				cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
				1.0, // Scale
				cv::Scalar(0, 0, 255), // BGR Color
				1, // Line Thickness (Optional)
				CV_AA); // Anti-alias (Optional)

		} // end loop over contours 

		/*------------------------------------------------------------------------------------------*/
		/* End Tracking Code */

		//show live and wait for a key with timeout long enough to show images

		imshow("Live", frame);
		if (cv::waitKey(5) >= 0) { break; }
	}

	return 0;
}

