<<<<<<< HEAD
#define _CRT_SECURE_NO_WARNINGS

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define PI 3.141592

struct sPoint {
	double x, y;
};
struct sLine {
	double mx, my;
	double sx, sy;
};

Mat computeVDisparity(Mat img);
Mat computeUDisparity(Mat img);
Mat occupancygrid(Mat occ, Mat forocc, Mat obstacle);

unsigned char m_pseudoColorLUT[256][3]; ///< RGB pseudo color
static bool readStringList(const string& filename, vector<string>& l);
void MakePseudoColorLUT();
void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor);
bool find_in_samples(sPoint *samples, int no_samples, sPoint *data);
void get_samples(sPoint *samples, int no_samples, sPoint *data, int no_data);
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model);
double compute_distance(sLine &line, sPoint &x);
double model_verification(sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold);
double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold);
//int dispTohist( Mat *img, Mat *imghist, int nDisp, double *dDistance );
//void Thinning(Mat input, int row, int col);

int main()
{
	const char* img1_filename = "kv30l_0.bmp";
	const char* img2_filename = "kv30r_0.bmp";

	const char* intrinsic_filename = "intrinsics1.yml";
	const char* extrinsic_filename = "extrinsics1.yml";

	//image list load	
	string Left_imglist_filename = "./imgLeft_list.xml";
	string Right_imglist_filename = "./imgRight_list.xml";;
	string Left_imglist_path = "./Stereo_sample_video_L/";
	string Right_imglist_path = "./Stereo_sample_video_R/";
	vector<string> vLeftimglist, vRightimglist;
	int nimages;

	bool ok = readStringList(Left_imglist_filename, vLeftimglist);
	if (ok == false) { cout << "left file error" << endl; return -1; }
	ok = readStringList(Right_imglist_filename, vRightimglist);
	if (ok == false) { cout << "Right file error" << endl; return -1; }
	if (vLeftimglist.size() != vRightimglist.size()) { cout << "image file error" << endl; return -1; }
	else nimages = vLeftimglist.size();

	enum { STEREO_BM = 0, STEREO_SGBM = 1 };
	int alg = 0;
	int SADWindowSize = 9, numberOfDisparities = 48;
	bool no_display = false;
	float scale = 1.f;

	StereoBM bm;
	StereoSGBM sgbm;

	if (!img1_filename || !img2_filename)
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		return -1;
	}

	if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		return -1;
	}

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1 = imread(img1_filename, color_mode);
	Mat img2 = imread(img2_filename, color_mode);

	if (scale != 1.f)
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;

	if (intrinsic_filename)
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}

		Mat M1, D1, M2, D2;
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;

		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
			return -1;
		}

		Mat R, T, R1, P1, R2, P2;
		fs["R"] >> R;
		fs["T"] >> T;

		//cout << R << endl << T << endl;
		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	}

	//numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

	bm.state->preFilterCap = 31; // 전처리
	bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10; // SAD응답의 최소값
	bm.state->uniquenessRatio = 15; // 후처리 위해
	bm.state->speckleWindowSize = 400;//100;
	bm.state->speckleRange = 32; //32;
	bm.state->disp12MaxDiff = 1;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3; // sgbm size : 5~9 odd 

	int cn = img1.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;

	Mat disp, img1re, img2re;
	Mat disp8(480, 360, CV_8U, Scalar(0));
	MakePseudoColorLUT();
	double pt1y = 0;
	double pt2y = 0;
	double Ymx = 0, Ymy = 0, Ysx = 0, Ysy = 0;
	Mat img1rec;
	int qq = 1;
	bool bEscKey = false;
	vector<vector<Point>> rect(qq, vector<Point>());
	/////////////////////////////////////on-line////////////////////////////////////////
	for (int i = 0; i<nimages; i++){

		img1 = imread((Left_imglist_path + vLeftimglist[i]).c_str(), color_mode);	//c_str() 는 char*로의 타입 변환 함수
		img2 = imread((Right_imglist_path + vRightimglist[i]).c_str(), color_mode);

		resize(img1, img1re, Size(320, 240));
		resize(img2, img2re, Size(320, 240));
		static int no_data = 400;
		int64 t = getTickCount();

		if (alg == STEREO_BM)
			bm(img1, img2, disp);
		else if (alg == STEREO_SGBM)
			sgbm(img1, img2, disp);

		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		if (!no_display)
		{
			imshow("left raw image", img1re);
			//imshow("disparity map", disp8);
			cvtColor(img1re, img1rec, COLOR_GRAY2BGR);

			Mat forocc;
			disp8.copyTo(forocc);

			Mat element = getStructuringElement(CV_SHAPE_RECT, Size(8, 8));	// 커널 생성
			Mat element1 = getStructuringElement(CV_SHAPE_RECT, Size(5, 5));	// 커널 생성
			Mat element2 = getStructuringElement(CV_SHAPE_RECT, Size(2, 2));	// 커널 생성
			Mat temp(360, 480, CV_8U, Scalar(0));
			
			Mat vvDisparity;
			Mat vDisparity = computeVDisparity(disp8);
			//threshold(vDisparity, vvDisparity, 10, 255, CV_THRESH_TOZERO);
			vDisparity.copyTo(vvDisparity);
			//imshow("VDisparity method", vDisparity);
			threshold(vDisparity, vDisparity, 30, 255, CV_THRESH_TOZERO);
			threshold(vDisparity, vvDisparity, 0, 255, CV_THRESH_BINARY);
			//imshow("VDisparity before", vDisparity);
			//Thinning(vDisparity, 360, 256);
			//imshow("VDisparity beforee", vvDisparity);

			//imshow("VDisparity thinning?", vDisparity);
			sPoint *data = new sPoint[no_data];
			int k = 0, cnt = 0;
			for (int j = 245; j>10; j--) {
				for (int i = 359; i>170; i--) {
					int d = vDisparity.at<unsigned char>(i, j);
					if (d != 0 && d<220 && k<no_data)
					{
						//cout<<"diap is :"<<d<<endl;
						data[k].x = j;
						data[k].y = i;
						k++;
						vDisparity.at<unsigned char>(i, j) = 255;
						cnt++;
					}
					if (k >= no_data)
						break;
					if (cnt == 3){
						cnt = 0;
						break;
					}
				}
				if (k >= no_data)
					break;
			}

			Mat vvvDisparity;
			cvtColor(vvDisparity, vvvDisparity, CV_GRAY2BGR);

			sLine ground;
			double cost = ransac_line_fitting(data, no_data, ground, 10);
			double ylim = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
			double slop = ground.my / ground.mx;
			if (30. < cost) {

				if (ylim >= 70 && ylim<180 && slop<1.3)
				{
					line(vvvDisparity, Point(0, ((ground.my*(-ground.sx)) / ground.mx) + ground.sy), Point(255, ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy), Scalar(0, 0, 255), 2);
					line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
					//cout<<((ground.my*(-ground.sx))/ground.mx)+ground.sy<<endl;

					pt1y = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
					pt2y = ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy;
					Ymx = ground.mx; Ymy = ground.my; Ysx = ground.sx; Ysy = ground.sy;
				}
				else
				{
					for (int k = no_data; k >= 25; k -= 30)
					{
						double cost = ransac_line_fitting(data, k, ground, 30);
						double ylim = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
						if (ylim >= 80 && ylim<160 && slop<1.4) // 처음 직선 검출 못했을 때 샘플개수 줄여보기
						{
							Ymx = ground.mx; Ymy = ground.my; Ysx = ground.sx; Ysy = ground.sy;
							line(vvvDisparity, Point(0, ((ground.my*(-ground.sx)) / ground.mx) + ground.sy), Point(255, ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy), Scalar(0, 0, 255), 2);
							line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
							break;
						}
						else if (k >= 250) // 직선 검출 못 했을 때
						{
							line(vvvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 2);
							line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
							break;
						}
					}
				}
			}

			//		for(int i=10;i<=245;i++)
			//{
			//	int d = vvDisparity.at<unsigned char>(((ground.my*(i-ground.sx))/ground.mx)+ground.sy,i);
			//	if( d== 255 )
			//	{
			//		vvvDisparity.at<>(((ground.my*(i-ground.sx))/ground.mx)+ground.sy,i) = Scalar(0,0,255);
			//	}
			//}

			imshow("Vdisparity linefit", vvvDisparity);

			Mat disp8uc, regroundc;
			Mat groundc(360, 480, CV_8UC3, CV_RGB(0, 0, 0));
			// 지면 나타내기
			// cout << "Ysy : " << Ysy << "Ysx : " << Ysx << endl;
			for (int i = 110; i<disp8.rows; i++) {
				for (int j = 48; j<disp8.cols; j++) {
					int d = disp8.at<unsigned char>(i, j);
					
					if (i >= (((Ymy*(d - Ysx)) / Ymx) + Ysy) - 12 && d != 0){
						disp8.at<unsigned char>(i, j) = 0;
						groundc.at<Vec3b>(i, j) = Vec3b(255, 84, 75);
					}
				}
			}
			resize(groundc, regroundc, Size(320, 240));
			//imshow("groundc",groundc);
			//imshow("regroundc",regroundc);
			//imshow("ground erase?",disp8);

			Mat result;

			// 윗면 없애기
			for (int i = 0; i<143; i++) {
				for (int j = 48; j<disp8.cols; j++) {
					int d = disp8.at<unsigned char>(i, j);
					if (i <= ((-0.685*d) + 147.4) && d != 0){
						disp8.at<unsigned char>(i, j) = 0;
						//groundc.at<Vec3b>(i,j)=Vec3b(255,84,75); 
					}
				}
			}

			Mat disp8u;
			//threshold(disp8, disp8u, 63.75, 255, CV_THRESH_TOZERO); // 25m
			//resize(disp8u,disp8uc,Size(320,240));

			//cvtColor(disp8uc,disp8uc,CV_GRAY2BGR);
			//Mat uDisparity = computeUDisparity(disp8u); 
			//imshow("disp filtering",disp8);
			//threshold(uDisparity, uDisparity, 15, 255, CV_THRESH_BINARY);
			//imshow("UDisparity before", uDisparity);
			morphologyEx(disp8, temp, CV_MOP_CLOSE, element1);	// 모폴로지 닫기	
			//imshow("filtering image", temp);
			//	threshold(disp8, disp8u, 79.68, 255, CV_THRESH_BINARY); // 25m
			//
			//	vector<vector<Point>> contours;
			//	 // 외곽선 벡터 , 외부 외곽선 검색, 각 외곽선의 모든 화소 탐색
			//	findContours(disp8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
			//		
			// int cmin= 300;  // 최소 외곽선 길이
			// int cmax= 10000; // 최대 외곽선 길이
			//
			// vector<vector<Point>>::const_iterator itc= contours.begin();
			//
			// while (itc!=contours.end()) {
			//  if (itc->size() < cmin || itc->size() > cmax)
			//		itc= contours.erase(itc);
			//  else 
			//   ++itc;
			// }
			//
			// // 원본 영상 내 외곽선 그리기
			//
			// //Mat original= imgThres;
			// // 모든 외곽선 그리기, 하얗게, 두께를 2로
			////	drawContours(original,contours, -1, Scalar(255), 2);      
			//
			// Rect r;
			// for(int i=0;i<contours.size();i++)
			// {
			//	  r= boundingRect(Mat(contours[i]));
			// 	 rectangle(disp8, r, Scalar(255), 2);	//disparity에 그 사각형 집어넣기
			// 	 rectangle(temp, r, Scalar(255), 2);	//disparity에 그 사각형 집어넣기
			//
			//	 int x=r.x+r.width/2;	//x는 사각형 가로 가운데
			//	 int y=(r.y+r.height/2)-4;	//y는 사각형 세로 가운데 -4
			//	 int sumd=0;
			//	 int k=0;
			//
			//	for(int i=0;i<8;i++)
			//	{
			//		int d=temp.at<unsigned char>(y,x);
			//		if(d>30 && d<220)
			//		{
			//			sumd+=d;	//d 값 누적
			//			k++;
			//		}
			//			//cout<<x<<"            "<<y<<"          "<<d<<"         "<<sumd<<endl;
			//		y++;
			//	}
			//	int DD=sumd/k;
			//	double distance = 0.25*900/DD*255/48;	// 밝기*(numberofdisp/255) = disparity
			//
			//	char dis_str[20];
			//	sprintf_s(dis_str, 20, "distance : %.1lf", distance);
			//
			//	putText(disp8,dis_str,Point(r.x,r.y-5), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255), 1.5);
			//
			// } 
			// imshow("color", disp8);

			Mat tempc, retempc, rectt, obs;
			cvtColor(temp, tempc, CV_GRAY2BGR);
			cvtPseudoColorImage(temp, tempc);

			threshold(temp, obs, 59.77, 255, CV_THRESH_TOZERO);
			resize(tempc, retempc, Size(320, 240));
			resize(temp, rectt, Size(320, 240));
			//imshow("color", retempc);

			add(retempc, regroundc, retempc);
			addWeighted(img1rec, 0.5, retempc, 0.5, 0.0, result);

			imshow("filtering result", result);
			threshold(rectt, rectt, 59.77, 255, CV_THRESH_TOZERO); // 20m
			// 클러스터링 해보기
			for (int v = rectt.cols - 4; v >= 30; v -= 3){
				int kk = 0;
				for (int u = rectt.rows - 1; u >= 0; u--){
					if (kk == 1 || kk == 2)
						break;
					float d = rectt.at<unsigned char>(u, v);
					if (d > 59.77 && d<200 && kk == 0){

						rect[0].push_back(Point(v, u));
						kk++;
						for (int uu = 0; uu<u - 10; uu++)
						{
							int ud = rectt.at<unsigned char>(uu, v);
							if (kk == 2)
								break;
							if (ud > 59.77 && ud<200){
								rect[0].push_back(Point(v - 3, uu));
								kk++;
							}
						}
					}
				}
			}
			Mat recttc;

			int color1, color2, color3;
			int loc = 0;
			int check = 0;
			//cout<<rect[qq-1][0]<<"     "<<rect[qq-1][1]<<rect[qq-1][2]<<endl;
			cvtColor(rectt, recttc, CV_GRAY2BGR);
			for (int k = 0; rect[0][k].x >= 37; k++){
				if ((rect[0][k].y - rect[0][k + 1].y)>12 && (rect[0][k].x - rect[0][k + 1].x) == 3){
					check++;

					if (check == 1){
						color1 = 255 - ((255 * rect[0][k].x) / 320);
						color2 = ((255 * rect[0][k].x) / 320);
						color3 = 255 - ((255 * abs(rect[0][k].x - 160)) / 160);
						rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
						loc = rect[0][k].x;
					}
					else
					{
						if ((loc - rect[0][k].x) == 3){
							rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
							loc = rect[0][k].x;
						}
						else{
							color1 = 255 - ((255 * rect[0][k].x) / 320);
							color2 = ((255 * rect[0][k].x) / 320);
							color3 = 255 - ((255 * abs(rect[0][k].x - 160)) / 160);
							rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
							loc = rect[0][k].x;
						}
					}
				}
			}
			rect[0].clear();

			Mat stixel;
			add(img1rec, recttc, stixel);
			//addWeighted(img1rec,0.3,recttc,0.7,0.0, stixel);
			imshow("stixel try", stixel);
			Mat occ(423, 480, CV_8UC3, Scalar(0));
			occupancygrid(occ, forocc, obs);
			imshow("occupancy grid", occ);

			t = getTickCount() - t;
			cout << "Time : " << t * 1000 / getTickFrequency() << " " << endl;
			delete [] data;
			waitKey(1);

			int nKey;
			while (!bEscKey)
			{
				cout << "press ESC key to start video" << endl;
				nKey = waitKey(0);
				switch (nKey)
				{
				case 27:
					bEscKey = true;
					break;
				}
			}
		}
		//} //while end;
	}
	return 0;
}

void MakePseudoColorLUT()
{
	int b = 125;
	int g = 0;
	int r = 0;

	int idx = 0;

	int mode = 0;
	// mode = 0 : increasing 'b'
	// mode = 1 : increasing 'g'
	// mode = 2 : decreasing 'b'
	// mode = 3 : increasing 'r'
	// mode = 4 : decreasing 'g'
	// mode = 5 : decreasing 'r'

	while (1)
	{
		m_pseudoColorLUT[idx][0] = r;
		m_pseudoColorLUT[idx][1] = g;
		m_pseudoColorLUT[idx][2] = b;

		if (b == 255 && g == 0 && r == 0)
			mode = 1;
		else if (b == 255 && g == 255 && r == 0)
			mode = 2;
		else if (b == 0 && g == 255 && r == 0)
			mode = 3;
		else if (b == 0 && g == 255 && r == 255)
			mode = 4;
		else if (b == 0 && g == 0 && r == 255)
			mode = 5;

		switch (mode)
		{
		case 0: b += 5; break;
		case 1: g += 5; break;
		case 2: b -= 5; break;
		case 3: r += 5; break;
		case 4: g -= 5; break;
		case 5: r -= 5; break;
		default: break;
		}

		if (idx == 255)
			break;

		idx++;
	}
}

void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor)
{
	for (int i = 0; i<srcGray.rows; i++)
	{
		for (int j = 0; j<srcGray.cols; j++)
		{
			unsigned char val = srcGray.data[i*srcGray.cols + j];
			if (val == 0) continue;
			dstColor.data[(i*srcGray.cols + j) * 3 + 0] = m_pseudoColorLUT[val][0];
			dstColor.data[(i*srcGray.cols + j) * 3 + 1] = m_pseudoColorLUT[val][1];
			dstColor.data[(i*srcGray.cols + j) * 3 + 2] = m_pseudoColorLUT[val][2];
		}
	}
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}
/*
int dispTohist( Mat *img, Mat *imghist, int nDisp, double *dDistance )
{
/// Establish the number of bins
int histSize = 256;

/// Set the ranges ( for B,G,R) )
float range[] = { 0, 256 };
const float* histRange = { range };

bool uniform = true; bool accumulate = false;

Mat hist;

/// Compute the histograms:
calcHist(img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
hist.at<float>(0, 1) = 0;
double maxVal = 0, minVal = 0;
minMaxLoc(hist, &minVal, &maxVal, 0, 0);
Mat histImg(256, 256, CV_8U, Scalar(255));

int hpt = static_cast<int>(0.9 * 256);
int disp_of_object = 0;

for (int h = 0; h<256; h++)// 208 : 5m
{
if (h<80 || h>250) hist.at<float>(h) = 0; // 80 = 20
float binVal = hist.at<float>(h);
disp_of_object = h;
int intensity = static_cast<int>(binVal*hpt / maxVal);
line(histImg, Point(h, 255), Point(h, 255 - intensity), Scalar::all(0));
}
*dDistance = (double)((0.25*1200) / ((double)(disp_of_object)*(double)nDisp / 255));

*imghist = histImg;
return 0;
}
*/
Mat computeVDisparity(Mat img)
{
	int maxDisp = 256;
	Mat vDisp(img.rows, maxDisp, CV_8U, Scalar(0));
	for (int u = 0; u<img.rows; u++){
		uchar* ptrA = img.ptr<unsigned char>(u);
		for (int v = 0; v<img.cols; v++){
			if (ptrA[v] != 0 && ptrA[v] != 255)
			{
				vDisp.at<unsigned char>(u, ptrA[v]) += 1;
			}
		}
	}
	return vDisp;
}

Mat computeUDisparity(Mat img)
{
	int maxDisp = 256;
	Mat uDisp(maxDisp, img.cols, CV_8U, Scalar(0));
	for (int u = 0; u<img.rows; u++){
		uchar* ptrA = img.ptr<unsigned char>(u);
		for (int v = 0; v<img.cols; v++){
			if (ptrA[v] != 0 && ptrA[v] != 255)
			{
				uDisp.at<unsigned char>(ptrA[v], v) += 1;
			}
		}
	}
	return uDisp;
}
Mat occupancygrid(Mat occ, Mat forocc, Mat obstacle) // forocc = disp img, obs = 장애물, 지면만 detection된 disp img
{
	Mat occupancyimg(423, 480, CV_8U, Scalar(0));
	cvtColor(occupancyimg, occ, CV_GRAY2BGR);

	ellipse(occ, Point(-2, 210.65), Size(33, 25), 0, 0, 360, CV_RGB(15, 0, 126), -1, 8);
	ellipse(occ, Point(-20, 203.65), Size(33, 5), 0, 0, 360, Scalar(180, 180, 180), -1, 8);
	ellipse(occ, Point(-20, 218.5), Size(33, 5), 0, 0, 360, Scalar(180, 180, 180), -1, 8);

	Rect e;
	e.x = 0; e.y = 201.5; e.width = 14; e.height = 20;
	rectangle(occ, e, Scalar(180, 180, 180), -1);
	ellipse(occ, Point(-28, 211.5), Size(33, 22), 0, 0, 360, CV_RGB(15, 0, 126), -1, 8);

	Point pt[2][4];
	pt[0][0] = Point(0, 0);
	pt[0][1] = Point(480, 0);
	pt[0][2] = Point(30, 197.5);
	pt[0][3] = Point(0, 197.5);
	pt[1][0] = Point(0, 224.5);
	pt[1][1] = Point(30, 224.5);
	pt[1][2] = Point(480, 423);
	pt[1][3] = Point(0, 423);

	const Point* ppt[1] = { pt[0] };
	const Point* ppt2[2] = { pt[1] };
	int npt[] = { 4 };
	fillPoly(occ, ppt, npt, 1, Scalar(0, 0, 0), 8, 0);
	fillPoly(occ, ppt2, npt, 1, Scalar(0, 0, 0), 8, 0);

	putText(occ, "10m", Point(148.35, 25), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 1.8);
	putText(occ, "20m", Point(307.9, 25), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 1.8);

	for (int u = 0; u<forocc.cols; u++) {
		for (int v = 0; v<forocc.rows; v++) {
			int dd = forocc.at<unsigned char>(v, u);
			double distance = 0.25 * 900 / dd * 255 / 48;
			float y = (480 * distance) / 30;
			float m = abs(v - 211.5)*0.32 / 211.5;
			float n = abs(u - 240)*0.44 / 240;

			if (dd>0 && dd<210 && y<480)
			{
				//		  //float disT = sqrt(powf(distance, 2) / (powf(m, 2) + powf(n, 2) + 1));
				float disT = sqrt(powf(distance, 2) - powf(m, 2) - powf(n, 2));
				float y = (480 * disT) / 30;
				//	  
				occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(0, 0, 160);
				//occ.at<Vec3b>(211.5-(y*0.44)+(0.44*2*y*(u-48))/432,y+1) = Vec3b(0,0,255);
				//occ.at<Vec3b>(211.5-(y*0.44)+(0.44*2*y*(u-48))/432,y+2) = Vec3b(0,0,255);
			}
		}
	}

	for (int u = 0; u<obstacle.cols; u++) {
		for (int v = 170; v<obstacle.rows; v++) {
			int dd = obstacle.at<unsigned char>(v, u);
			double distance = 0.25 * 900 / dd * 255 / 48;
			float y = (480 * distance) / 30;

			if (dd>0 && y<480)
			{
				if (distance<15)
				{
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(59, 176, 255);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 1) = Vec3b(59, 176, 255);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 2) = Vec3b(59, 176, 255);
				}
				else
				{
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(255, 0, 0);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 1) = Vec3b(255, 0, 0);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 2) = Vec3b(255, 0, 0);
				}
			}
		}
	}
	return occ;
}
bool find_in_samples(sPoint *samples, int no_samples, sPoint *data)
{
	for (int i = 0; i<no_samples; ++i) {
		if (samples[i].x == data->x && samples[i].y == data->y) {
			return true;
		}
	}
	return false;
}
void get_samples(sPoint *samples, int no_samples, sPoint *data, int no_data)
{
	// 데이터에서 중복되지 않게 N개의 무작위 셈플을 채취한다.
	for (int i = 0; i<no_samples;) {
		int j = rand() % no_data;

		if (!find_in_samples(samples, i, &data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model)
{
	// PCA 방식으로 직선 모델의 파라메터를 예측한다.

	double sx = 0, sy = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw = 0;

	for (int i = 0; i<no_samples; ++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx += x;
		sy += y;
		sxx += x*x;
		sxy += x*y;
		syy += y*y;
		sw += 1;
	}

	//variance;
	double vxx = (sxx - sx*sx / sw) / sw;
	double vxy = (sxy - sx*sy / sw) / sw;
	double vyy = (syy - sy*sy / sw) / sw;

	//principal axis
	double theta = atan2(2 * vxy, vxx - vyy) / 2;

	model.mx = cos(theta);
	model.my = sin(theta);

	//center of mass(xc, yc)
	model.sx = sx / sw;
	model.sy = sy / sw;

	//직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	return 1;
}
double compute_distance(sLine &line, sPoint &x)
{
	// 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.

	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx) / sqrt(line.mx*line.mx + line.my*line.my);
}
double model_verification(sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;
	double cost = 0.;

	for (int i = 0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		double distance = compute_distance(estimated_model, data[i]);

		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}
	return cost;
}
double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold)
{
	const int no_samples = 2;

	if (no_data < no_samples) {
		return 0;
	}

	sPoint *samples = new sPoint[no_samples];

	int no_inliers = 0;
	sPoint *inliers = new sPoint[no_data];

	sLine estimated_model;
	double max_cost = 0.;

	int max_iteration = (int)(1 + log(1. - 0.99) / log(1. - pow(0.6, no_samples)));

	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis
		// 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
		get_samples(samples, no_samples, data, no_data);

		// 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
		compute_model_parameter(samples, no_samples, estimated_model);
		// 2. Verification
		// 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
		double cost = model_verification(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);
		// 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
		if (max_cost < cost) {
			max_cost = cost;
			compute_model_parameter(inliers, no_inliers, model);
		}
	}
	delete[] samples;
	delete[] inliers;

	return max_cost;
=======
#define _CRT_SECURE_NO_WARNINGS

#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define PI 3.141592

struct sPoint {
	double x, y;
};
struct sLine {
	double mx, my;
	double sx, sy;
};

Mat computeVDisparity(Mat img);
Mat computeUDisparity(Mat img);
Mat occupancygrid(Mat occ, Mat forocc, Mat obstacle);

unsigned char m_pseudoColorLUT[256][3]; ///< RGB pseudo color
static bool readStringList(const string& filename, vector<string>& l);
void MakePseudoColorLUT();
void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor);
bool find_in_samples(sPoint *samples, int no_samples, sPoint *data);
void get_samples(sPoint *samples, int no_samples, sPoint *data, int no_data);
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model);
double compute_distance(sLine &line, sPoint &x);
double model_verification(sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold);
double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold);
//int dispTohist( Mat *img, Mat *imghist, int nDisp, double *dDistance );
//void Thinning(Mat input, int row, int col);

int main()
{
	const char* img1_filename = "kv30l_0.bmp";
	const char* img2_filename = "kv30r_0.bmp";

	const char* intrinsic_filename = "intrinsics1.yml";
	const char* extrinsic_filename = "extrinsics1.yml";

	//image list load	
	string Left_imglist_filename = "./imgLeft_list.xml";
	string Right_imglist_filename = "./imgRight_list.xml";;
	string Left_imglist_path = "./Stereo_sample_video_L/";
	string Right_imglist_path = "./Stereo_sample_video_R/";
	vector<string> vLeftimglist, vRightimglist;
	int nimages;

	bool ok = readStringList(Left_imglist_filename, vLeftimglist);
	if (ok == false) { cout << "left file error" << endl; return -1; }
	ok = readStringList(Right_imglist_filename, vRightimglist);
	if (ok == false) { cout << "Right file error" << endl; return -1; }
	if (vLeftimglist.size() != vRightimglist.size()) { cout << "image file error" << endl; return -1; }
	else nimages = vLeftimglist.size();

	enum { STEREO_BM = 0, STEREO_SGBM = 1 };
	int alg = 0;
	int SADWindowSize = 9, numberOfDisparities = 48;
	bool no_display = false;
	float scale = 1.f;

	StereoBM bm;
	StereoSGBM sgbm;

	if (!img1_filename || !img2_filename)
	{
		printf("Command-line parameter error: both left and right images must be specified\n");
		return -1;
	}

	if ((intrinsic_filename != 0) ^ (extrinsic_filename != 0))
	{
		printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
		return -1;
	}

	int color_mode = alg == STEREO_BM ? 0 : -1;
	Mat img1 = imread(img1_filename, color_mode);
	Mat img2 = imread(img2_filename, color_mode);

	if (scale != 1.f)
	{
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;

	if (intrinsic_filename)
	{
		// reading intrinsic parameters
		FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", intrinsic_filename);
			return -1;
		}

		Mat M1, D1, M2, D2;
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		M1 *= scale;
		M2 *= scale;

		fs.open(extrinsic_filename, CV_STORAGE_READ);
		if (!fs.isOpened())
		{
			printf("Failed to open file %s\n", extrinsic_filename);
			return -1;
		}

		Mat R, T, R1, P1, R2, P2;
		fs["R"] >> R;
		fs["T"] >> T;

		//cout << R << endl << T << endl;
		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	}

	//numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

	bm.state->preFilterCap = 31; // 전처리
	bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10; // SAD응답의 최소값
	bm.state->uniquenessRatio = 15; // 후처리 위해
	bm.state->speckleWindowSize = 400;//100;
	bm.state->speckleRange = 32; //32;
	bm.state->disp12MaxDiff = 1;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3; // sgbm size : 5~9 odd 

	int cn = img1.channels();

	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;

	Mat disp, img1re, img2re;
	Mat disp8(480, 360, CV_8U, Scalar(0));
	MakePseudoColorLUT();
	double pt1y = 0;
	double pt2y = 0;
	double Ymx = 0, Ymy = 0, Ysx = 0, Ysy = 0;
	Mat img1rec;
	int qq = 1;
	bool bEscKey = false;
	vector<vector<Point>> rect(qq, vector<Point>());
	/////////////////////////////////////on-line////////////////////////////////////////
	for (int i = 0; i<nimages; i++){

		img1 = imread((Left_imglist_path + vLeftimglist[i]).c_str(), color_mode);	//c_str() 는 char*로의 타입 변환 함수
		img2 = imread((Right_imglist_path + vRightimglist[i]).c_str(), color_mode);

		resize(img1, img1re, Size(320, 240));
		resize(img2, img2re, Size(320, 240));
		static int no_data = 400;
		int64 t = getTickCount();

		if (alg == STEREO_BM)
			bm(img1, img2, disp);
		else if (alg == STEREO_SGBM)
			sgbm(img1, img2, disp);

		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		if (!no_display)
		{
			imshow("left raw image", img1re);
			//imshow("disparity map", disp8);
			cvtColor(img1re, img1rec, COLOR_GRAY2BGR);

			Mat forocc;
			disp8.copyTo(forocc);

			Mat element = getStructuringElement(CV_SHAPE_RECT, Size(8, 8));	// 커널 생성
			Mat element1 = getStructuringElement(CV_SHAPE_RECT, Size(5, 5));	// 커널 생성
			Mat element2 = getStructuringElement(CV_SHAPE_RECT, Size(2, 2));	// 커널 생성
			Mat temp(360, 480, CV_8U, Scalar(0));
			
			Mat vvDisparity;
			Mat vDisparity = computeVDisparity(disp8);
			//threshold(vDisparity, vvDisparity, 10, 255, CV_THRESH_TOZERO);
			vDisparity.copyTo(vvDisparity);
			//imshow("VDisparity method", vDisparity);
			threshold(vDisparity, vDisparity, 30, 255, CV_THRESH_TOZERO);
			threshold(vDisparity, vvDisparity, 0, 255, CV_THRESH_BINARY);
			//imshow("VDisparity before", vDisparity);
			//Thinning(vDisparity, 360, 256);
			//imshow("VDisparity beforee", vvDisparity);

			//imshow("VDisparity thinning?", vDisparity);
			sPoint *data = new sPoint[no_data];
			int k = 0, cnt = 0;
			for (int j = 245; j>10; j--) {
				for (int i = 359; i>170; i--) {
					int d = vDisparity.at<unsigned char>(i, j);
					if (d != 0 && d<220 && k<no_data)
					{
						//cout<<"diap is :"<<d<<endl;
						data[k].x = j;
						data[k].y = i;
						k++;
						vDisparity.at<unsigned char>(i, j) = 255;
						cnt++;
					}
					if (k >= no_data)
						break;
					if (cnt == 3){
						cnt = 0;
						break;
					}
				}
				if (k >= no_data)
					break;
			}

			Mat vvvDisparity;
			cvtColor(vvDisparity, vvvDisparity, CV_GRAY2BGR);

			sLine ground;
			double cost = ransac_line_fitting(data, no_data, ground, 10);
			double ylim = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
			double slop = ground.my / ground.mx;
			if (30. < cost) {

				if (ylim >= 70 && ylim<180 && slop<1.3)
				{
					line(vvvDisparity, Point(0, ((ground.my*(-ground.sx)) / ground.mx) + ground.sy), Point(255, ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy), Scalar(0, 0, 255), 2);
					line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
					//cout<<((ground.my*(-ground.sx))/ground.mx)+ground.sy<<endl;

					pt1y = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
					pt2y = ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy;
					Ymx = ground.mx; Ymy = ground.my; Ysx = ground.sx; Ysy = ground.sy;
				}
				else
				{
					for (int k = no_data; k >= 25; k -= 30)
					{
						double cost = ransac_line_fitting(data, k, ground, 30);
						double ylim = ((ground.my*(-ground.sx)) / ground.mx) + ground.sy;
						if (ylim >= 80 && ylim<160 && slop<1.4) // 처음 직선 검출 못했을 때 샘플개수 줄여보기
						{
							Ymx = ground.mx; Ymy = ground.my; Ysx = ground.sx; Ysy = ground.sy;
							line(vvvDisparity, Point(0, ((ground.my*(-ground.sx)) / ground.mx) + ground.sy), Point(255, ((ground.my*(255 - ground.sx)) / ground.mx) + ground.sy), Scalar(0, 0, 255), 2);
							line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
							break;
						}
						else if (k >= 250) // 직선 검출 못 했을 때
						{
							line(vvvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 2);
							line(vvDisparity, Point(0, pt1y), Point(255, pt2y), Scalar(0, 0, 255), 1);
							break;
						}
					}
				}
			}

			//		for(int i=10;i<=245;i++)
			//{
			//	int d = vvDisparity.at<unsigned char>(((ground.my*(i-ground.sx))/ground.mx)+ground.sy,i);
			//	if( d== 255 )
			//	{
			//		vvvDisparity.at<>(((ground.my*(i-ground.sx))/ground.mx)+ground.sy,i) = Scalar(0,0,255);
			//	}
			//}

			imshow("Vdisparity linefit", vvvDisparity);

			Mat disp8uc, regroundc;
			Mat groundc(360, 480, CV_8UC3, CV_RGB(0, 0, 0));
			// 지면 나타내기
			// cout << "Ysy : " << Ysy << "Ysx : " << Ysx << endl;
			for (int i = 110; i<disp8.rows; i++) {
				for (int j = 48; j<disp8.cols; j++) {
					int d = disp8.at<unsigned char>(i, j);
					
					if (i >= (((Ymy*(d - Ysx)) / Ymx) + Ysy) - 12 && d != 0){
						disp8.at<unsigned char>(i, j) = 0;
						groundc.at<Vec3b>(i, j) = Vec3b(255, 84, 75);
					}
				}
			}
			resize(groundc, regroundc, Size(320, 240));
			//imshow("groundc",groundc);
			//imshow("regroundc",regroundc);
			//imshow("ground erase?",disp8);

			Mat result;

			// 윗면 없애기
			for (int i = 0; i<143; i++) {
				for (int j = 48; j<disp8.cols; j++) {
					int d = disp8.at<unsigned char>(i, j);
					if (i <= ((-0.685*d) + 147.4) && d != 0){
						disp8.at<unsigned char>(i, j) = 0;
						//groundc.at<Vec3b>(i,j)=Vec3b(255,84,75); 
					}
				}
			}

			Mat disp8u;
			//threshold(disp8, disp8u, 63.75, 255, CV_THRESH_TOZERO); // 25m
			//resize(disp8u,disp8uc,Size(320,240));

			//cvtColor(disp8uc,disp8uc,CV_GRAY2BGR);
			//Mat uDisparity = computeUDisparity(disp8u); 
			//imshow("disp filtering",disp8);
			//threshold(uDisparity, uDisparity, 15, 255, CV_THRESH_BINARY);
			//imshow("UDisparity before", uDisparity);
			morphologyEx(disp8, temp, CV_MOP_CLOSE, element1);	// 모폴로지 닫기	
			//imshow("filtering image", temp);
			//	threshold(disp8, disp8u, 79.68, 255, CV_THRESH_BINARY); // 25m
			//
			//	vector<vector<Point>> contours;
			//	 // 외곽선 벡터 , 외부 외곽선 검색, 각 외곽선의 모든 화소 탐색
			//	findContours(disp8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); 
			//		
			// int cmin= 300;  // 최소 외곽선 길이
			// int cmax= 10000; // 최대 외곽선 길이
			//
			// vector<vector<Point>>::const_iterator itc= contours.begin();
			//
			// while (itc!=contours.end()) {
			//  if (itc->size() < cmin || itc->size() > cmax)
			//		itc= contours.erase(itc);
			//  else 
			//   ++itc;
			// }
			//
			// // 원본 영상 내 외곽선 그리기
			//
			// //Mat original= imgThres;
			// // 모든 외곽선 그리기, 하얗게, 두께를 2로
			////	drawContours(original,contours, -1, Scalar(255), 2);      
			//
			// Rect r;
			// for(int i=0;i<contours.size();i++)
			// {
			//	  r= boundingRect(Mat(contours[i]));
			// 	 rectangle(disp8, r, Scalar(255), 2);	//disparity에 그 사각형 집어넣기
			// 	 rectangle(temp, r, Scalar(255), 2);	//disparity에 그 사각형 집어넣기
			//
			//	 int x=r.x+r.width/2;	//x는 사각형 가로 가운데
			//	 int y=(r.y+r.height/2)-4;	//y는 사각형 세로 가운데 -4
			//	 int sumd=0;
			//	 int k=0;
			//
			//	for(int i=0;i<8;i++)
			//	{
			//		int d=temp.at<unsigned char>(y,x);
			//		if(d>30 && d<220)
			//		{
			//			sumd+=d;	//d 값 누적
			//			k++;
			//		}
			//			//cout<<x<<"            "<<y<<"          "<<d<<"         "<<sumd<<endl;
			//		y++;
			//	}
			//	int DD=sumd/k;
			//	double distance = 0.25*900/DD*255/48;	// 밝기*(numberofdisp/255) = disparity
			//
			//	char dis_str[20];
			//	sprintf_s(dis_str, 20, "distance : %.1lf", distance);
			//
			//	putText(disp8,dis_str,Point(r.x,r.y-5), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255), 1.5);
			//
			// } 
			// imshow("color", disp8);

			Mat tempc, retempc, rectt, obs;
			cvtColor(temp, tempc, CV_GRAY2BGR);
			cvtPseudoColorImage(temp, tempc);

			threshold(temp, obs, 59.77, 255, CV_THRESH_TOZERO);
			resize(tempc, retempc, Size(320, 240));
			resize(temp, rectt, Size(320, 240));
			//imshow("color", retempc);

			add(retempc, regroundc, retempc);
			addWeighted(img1rec, 0.5, retempc, 0.5, 0.0, result);

			imshow("filtering result", result);
			threshold(rectt, rectt, 59.77, 255, CV_THRESH_TOZERO); // 20m
			// 클러스터링 해보기
			for (int v = rectt.cols - 4; v >= 30; v -= 3){
				int kk = 0;
				for (int u = rectt.rows - 1; u >= 0; u--){
					if (kk == 1 || kk == 2)
						break;
					float d = rectt.at<unsigned char>(u, v);
					if (d > 59.77 && d<200 && kk == 0){

						rect[0].push_back(Point(v, u));
						kk++;
						for (int uu = 0; uu<u - 10; uu++)
						{
							int ud = rectt.at<unsigned char>(uu, v);
							if (kk == 2)
								break;
							if (ud > 59.77 && ud<200){
								rect[0].push_back(Point(v - 3, uu));
								kk++;
							}
						}
					}
				}
			}
			Mat recttc;

			int color1, color2, color3;
			int loc = 0;
			int check = 0;
			//cout<<rect[qq-1][0]<<"     "<<rect[qq-1][1]<<rect[qq-1][2]<<endl;
			cvtColor(rectt, recttc, CV_GRAY2BGR);
			for (int k = 0; rect[0][k].x >= 37; k++){
				if ((rect[0][k].y - rect[0][k + 1].y)>12 && (rect[0][k].x - rect[0][k + 1].x) == 3){
					check++;

					if (check == 1){
						color1 = 255 - ((255 * rect[0][k].x) / 320);
						color2 = ((255 * rect[0][k].x) / 320);
						color3 = 255 - ((255 * abs(rect[0][k].x - 160)) / 160);
						rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
						loc = rect[0][k].x;
					}
					else
					{
						if ((loc - rect[0][k].x) == 3){
							rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
							loc = rect[0][k].x;
						}
						else{
							color1 = 255 - ((255 * rect[0][k].x) / 320);
							color2 = ((255 * rect[0][k].x) / 320);
							color3 = 255 - ((255 * abs(rect[0][k].x - 160)) / 160);
							rectangle(recttc, rect[0][k], rect[0][k + 1], Scalar(color1, color2, color3), 1);
							loc = rect[0][k].x;
						}
					}
				}
			}
			rect[0].clear();

			Mat stixel;
			add(img1rec, recttc, stixel);
			//addWeighted(img1rec,0.3,recttc,0.7,0.0, stixel);
			imshow("stixel try", stixel);
			Mat occ(423, 480, CV_8UC3, Scalar(0));
			occupancygrid(occ, forocc, obs);
			imshow("occupancy grid", occ);

			t = getTickCount() - t;
			cout << "Time : " << t * 1000 / getTickFrequency() << " " << endl;
			delete [] data;
			waitKey(1);

			int nKey;
			while (!bEscKey)
			{
				cout << "press ESC key to start video" << endl;
				nKey = waitKey(0);
				switch (nKey)
				{
				case 27:
					bEscKey = true;
					break;
				}
			}
		}
		//} //while end;
	}
	return 0;
}

void MakePseudoColorLUT()
{
	int b = 125;
	int g = 0;
	int r = 0;

	int idx = 0;

	int mode = 0;
	// mode = 0 : increasing 'b'
	// mode = 1 : increasing 'g'
	// mode = 2 : decreasing 'b'
	// mode = 3 : increasing 'r'
	// mode = 4 : decreasing 'g'
	// mode = 5 : decreasing 'r'

	while (1)
	{
		m_pseudoColorLUT[idx][0] = r;
		m_pseudoColorLUT[idx][1] = g;
		m_pseudoColorLUT[idx][2] = b;

		if (b == 255 && g == 0 && r == 0)
			mode = 1;
		else if (b == 255 && g == 255 && r == 0)
			mode = 2;
		else if (b == 0 && g == 255 && r == 0)
			mode = 3;
		else if (b == 0 && g == 255 && r == 255)
			mode = 4;
		else if (b == 0 && g == 0 && r == 255)
			mode = 5;

		switch (mode)
		{
		case 0: b += 5; break;
		case 1: g += 5; break;
		case 2: b -= 5; break;
		case 3: r += 5; break;
		case 4: g -= 5; break;
		case 5: r -= 5; break;
		default: break;
		}

		if (idx == 255)
			break;

		idx++;
	}
}

void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor)
{
	for (int i = 0; i<srcGray.rows; i++)
	{
		for (int j = 0; j<srcGray.cols; j++)
		{
			unsigned char val = srcGray.data[i*srcGray.cols + j];
			if (val == 0) continue;
			dstColor.data[(i*srcGray.cols + j) * 3 + 0] = m_pseudoColorLUT[val][0];
			dstColor.data[(i*srcGray.cols + j) * 3 + 1] = m_pseudoColorLUT[val][1];
			dstColor.data[(i*srcGray.cols + j) * 3 + 2] = m_pseudoColorLUT[val][2];
		}
	}
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}
/*
int dispTohist( Mat *img, Mat *imghist, int nDisp, double *dDistance )
{
/// Establish the number of bins
int histSize = 256;

/// Set the ranges ( for B,G,R) )
float range[] = { 0, 256 };
const float* histRange = { range };

bool uniform = true; bool accumulate = false;

Mat hist;

/// Compute the histograms:
calcHist(img, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
hist.at<float>(0, 1) = 0;
double maxVal = 0, minVal = 0;
minMaxLoc(hist, &minVal, &maxVal, 0, 0);
Mat histImg(256, 256, CV_8U, Scalar(255));

int hpt = static_cast<int>(0.9 * 256);
int disp_of_object = 0;

for (int h = 0; h<256; h++)// 208 : 5m
{
if (h<80 || h>250) hist.at<float>(h) = 0; // 80 = 20
float binVal = hist.at<float>(h);
disp_of_object = h;
int intensity = static_cast<int>(binVal*hpt / maxVal);
line(histImg, Point(h, 255), Point(h, 255 - intensity), Scalar::all(0));
}
*dDistance = (double)((0.25*1200) / ((double)(disp_of_object)*(double)nDisp / 255));

*imghist = histImg;
return 0;
}
*/
Mat computeVDisparity(Mat img)
{
	int maxDisp = 256;
	Mat vDisp(img.rows, maxDisp, CV_8U, Scalar(0));
	for (int u = 0; u<img.rows; u++){
		uchar* ptrA = img.ptr<unsigned char>(u);
		for (int v = 0; v<img.cols; v++){
			if (ptrA[v] != 0 && ptrA[v] != 255)
			{
				vDisp.at<unsigned char>(u, ptrA[v]) += 1;
			}
		}
	}
	return vDisp;
}

Mat computeUDisparity(Mat img)
{
	int maxDisp = 256;
	Mat uDisp(maxDisp, img.cols, CV_8U, Scalar(0));
	for (int u = 0; u<img.rows; u++){
		uchar* ptrA = img.ptr<unsigned char>(u);
		for (int v = 0; v<img.cols; v++){
			if (ptrA[v] != 0 && ptrA[v] != 255)
			{
				uDisp.at<unsigned char>(ptrA[v], v) += 1;
			}
		}
	}
	return uDisp;
}
Mat occupancygrid(Mat occ, Mat forocc, Mat obstacle) // forocc = disp img, obs = 장애물, 지면만 detection된 disp img
{
	Mat occupancyimg(423, 480, CV_8U, Scalar(0));
	cvtColor(occupancyimg, occ, CV_GRAY2BGR);

	ellipse(occ, Point(-2, 210.65), Size(33, 25), 0, 0, 360, CV_RGB(15, 0, 126), -1, 8);
	ellipse(occ, Point(-20, 203.65), Size(33, 5), 0, 0, 360, Scalar(180, 180, 180), -1, 8);
	ellipse(occ, Point(-20, 218.5), Size(33, 5), 0, 0, 360, Scalar(180, 180, 180), -1, 8);

	Rect e;
	e.x = 0; e.y = 201.5; e.width = 14; e.height = 20;
	rectangle(occ, e, Scalar(180, 180, 180), -1);
	ellipse(occ, Point(-28, 211.5), Size(33, 22), 0, 0, 360, CV_RGB(15, 0, 126), -1, 8);

	Point pt[2][4];
	pt[0][0] = Point(0, 0);
	pt[0][1] = Point(480, 0);
	pt[0][2] = Point(30, 197.5);
	pt[0][3] = Point(0, 197.5);
	pt[1][0] = Point(0, 224.5);
	pt[1][1] = Point(30, 224.5);
	pt[1][2] = Point(480, 423);
	pt[1][3] = Point(0, 423);

	const Point* ppt[1] = { pt[0] };
	const Point* ppt2[2] = { pt[1] };
	int npt[] = { 4 };
	fillPoly(occ, ppt, npt, 1, Scalar(0, 0, 0), 8, 0);
	fillPoly(occ, ppt2, npt, 1, Scalar(0, 0, 0), 8, 0);

	putText(occ, "10m", Point(148.35, 25), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 1.8);
	putText(occ, "20m", Point(307.9, 25), FONT_HERSHEY_PLAIN, 1, CV_RGB(255, 255, 255), 1.8);

	for (int u = 0; u<forocc.cols; u++) {
		for (int v = 0; v<forocc.rows; v++) {
			int dd = forocc.at<unsigned char>(v, u);
			double distance = 0.25 * 900 / dd * 255 / 48;
			float y = (480 * distance) / 30;
			float m = abs(v - 211.5)*0.32 / 211.5;
			float n = abs(u - 240)*0.44 / 240;

			if (dd>0 && dd<210 && y<480)
			{
				//		  //float disT = sqrt(powf(distance, 2) / (powf(m, 2) + powf(n, 2) + 1));
				float disT = sqrt(powf(distance, 2) - powf(m, 2) - powf(n, 2));
				float y = (480 * disT) / 30;
				//	  
				occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(0, 0, 160);
				//occ.at<Vec3b>(211.5-(y*0.44)+(0.44*2*y*(u-48))/432,y+1) = Vec3b(0,0,255);
				//occ.at<Vec3b>(211.5-(y*0.44)+(0.44*2*y*(u-48))/432,y+2) = Vec3b(0,0,255);
			}
		}
	}

	for (int u = 0; u<obstacle.cols; u++) {
		for (int v = 170; v<obstacle.rows; v++) {
			int dd = obstacle.at<unsigned char>(v, u);
			double distance = 0.25 * 900 / dd * 255 / 48;
			float y = (480 * distance) / 30;

			if (dd>0 && y<480)
			{
				if (distance<15)
				{
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(59, 176, 255);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 1) = Vec3b(59, 176, 255);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 2) = Vec3b(59, 176, 255);
				}
				else
				{
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y) = Vec3b(255, 0, 0);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 1) = Vec3b(255, 0, 0);
					occ.at<Vec3b>(211.5 - (y*0.44) + (0.44 * 2 * y*(u - 48)) / 432, y + 2) = Vec3b(255, 0, 0);
				}
			}
		}
	}
	return occ;
}
bool find_in_samples(sPoint *samples, int no_samples, sPoint *data)
{
	for (int i = 0; i<no_samples; ++i) {
		if (samples[i].x == data->x && samples[i].y == data->y) {
			return true;
		}
	}
	return false;
}
void get_samples(sPoint *samples, int no_samples, sPoint *data, int no_data)
{
	// 데이터에서 중복되지 않게 N개의 무작위 셈플을 채취한다.
	for (int i = 0; i<no_samples;) {
		int j = rand() % no_data;

		if (!find_in_samples(samples, i, &data[j])) {
			samples[i] = data[j];
			++i;
		}
	};
}
int compute_model_parameter(sPoint samples[], int no_samples, sLine &model)
{
	// PCA 방식으로 직선 모델의 파라메터를 예측한다.

	double sx = 0, sy = 0;
	double sxx = 0, syy = 0;
	double sxy = 0, sw = 0;

	for (int i = 0; i<no_samples; ++i)
	{
		double &x = samples[i].x;
		double &y = samples[i].y;

		sx += x;
		sy += y;
		sxx += x*x;
		sxy += x*y;
		syy += y*y;
		sw += 1;
	}

	//variance;
	double vxx = (sxx - sx*sx / sw) / sw;
	double vxy = (sxy - sx*sy / sw) / sw;
	double vyy = (syy - sy*sy / sw) / sw;

	//principal axis
	double theta = atan2(2 * vxy, vxx - vyy) / 2;

	model.mx = cos(theta);
	model.my = sin(theta);

	//center of mass(xc, yc)
	model.sx = sx / sw;
	model.sy = sy / sw;

	//직선의 방정식: sin(theta)*(x - sx) = cos(theta)*(y - sy);
	return 1;
}
double compute_distance(sLine &line, sPoint &x)
{
	// 한 점(x)로부터 직선(line)에 내린 수선의 길이(distance)를 계산한다.

	return fabs((x.x - line.sx)*line.my - (x.y - line.sy)*line.mx) / sqrt(line.mx*line.mx + line.my*line.my);
}
double model_verification(sPoint *inliers, int *no_inliers, sLine &estimated_model, sPoint *data, int no_data, double distance_threshold)
{
	*no_inliers = 0;
	double cost = 0.;

	for (int i = 0; i<no_data; i++){
		// 직선에 내린 수선의 길이를 계산한다.
		double distance = compute_distance(estimated_model, data[i]);

		// 예측된 모델에서 유효한 데이터인 경우, 유효한 데이터 집합에 더한다.
		if (distance < distance_threshold) {
			cost += 1.;

			inliers[*no_inliers] = data[i];
			++(*no_inliers);
		}
	}
	return cost;
}
double ransac_line_fitting(sPoint *data, int no_data, sLine &model, double distance_threshold)
{
	const int no_samples = 2;

	if (no_data < no_samples) {
		return 0;
	}

	sPoint *samples = new sPoint[no_samples];

	int no_inliers = 0;
	sPoint *inliers = new sPoint[no_data];

	sLine estimated_model;
	double max_cost = 0.;

	int max_iteration = (int)(1 + log(1. - 0.99) / log(1. - pow(0.6, no_samples)));

	for (int i = 0; i<max_iteration; i++) {
		// 1. hypothesis
		// 원본 데이터에서 임의로 N개의 셈플 데이터를 고른다.
		get_samples(samples, no_samples, data, no_data);

		// 이 데이터를 정상적인 데이터로 보고 모델 파라메터를 예측한다.
		compute_model_parameter(samples, no_samples, estimated_model);
		// 2. Verification
		// 원본 데이터가 예측된 모델에 잘 맞는지 검사한다.
		double cost = model_verification(inliers, &no_inliers, estimated_model, data, no_data, distance_threshold);
		// 만일 예측된 모델이 잘 맞는다면, 이 모델에 대한 유효한 데이터로 새로운 모델을 구한다.
		if (max_cost < cost) {
			max_cost = cost;
			compute_model_parameter(inliers, no_inliers, model);
		}
	}
	delete[] samples;
	delete[] inliers;

	return max_cost;
>>>>>>> a86ca9fe1334ccabce446898449615dfc9b00bc2
}