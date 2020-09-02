# MarkerTracker
Fiducial Marker Tracking with OpenCV

Notes:
1. Code can be run by running the MarkerTracker.cpp file, which is the driving file
2. This code was made using opencv version 3.4.10, in Visual Studio Community
3. The threshold may need to be adjusted (or a trackbar added) if you notice markers are hard to pick up from the camera
4. The camera intrinsics matrix and distortion coefficients were calculated separately with a personal computer.
   Use any standard camera calibration software from OpenCV website (or included with library) so that it works
   on your device. 
5. Special thanks to JHU CAMP Lab (Computer Aided Medical Procedures) for all the Pose Estimation code 
