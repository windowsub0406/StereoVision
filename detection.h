<<<<<<< HEAD
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

class Detection
{
public:

//	Detection();
	static bool readStringList( const string& filename, vector<string>& l );
	Mat removeground(Mat dispg);
	Mat removeupper(Mat dispu);
	int dispTohist(Mat *img, Mat *imghist, int nDisp,  double *dDistance); 
	Mat rectAnddistance(Mat road, Mat roadc);
	Mat occupancygrid(Mat occ, Mat dis);
	Mat lanedetection(Mat frame, int leftturn, int rightturn);
};
=======
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

class Detection
{
public:

//	Detection();
	static bool readStringList( const string& filename, vector<string>& l );
	Mat removeground(Mat dispg);
	Mat removeupper(Mat dispu);
	int dispTohist(Mat *img, Mat *imghist, int nDisp,  double *dDistance); 
	Mat rectAnddistance(Mat road, Mat roadc);
	Mat occupancygrid(Mat occ, Mat dis);
	Mat lanedetection(Mat frame, int leftturn, int rightturn);
};
>>>>>>> a86ca9fe1334ccabce446898449615dfc9b00bc2
