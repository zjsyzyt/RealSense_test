// RealSense_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
//#include "RSWrapper.hpp" 
#include <iostream>
#include "rs.hpp"

using namespace cv;
using namespace std;
using namespace rs2;

//void drawText(Mat & image);

//int main()
//{
//	cout << "Built with OpenCV " << CV_VERSION << endl;
//	Mat image;
//	VideoCapture capture;
//	capture.open(2);
//	if (capture.isOpened())
//	{
//		cout << "Capture is opened" << endl;
//		for (;;)
//		{
//			capture >> image;
//			if (image.empty())
//				break;
//			drawText(image);
//			imshow("Sample", image);
//			if (waitKey(10) >= 0)
//				break;
//		}
//	}
//	else
//	{
//		cout << "No capture" << endl; 321;
//		image = Mat::zeros(480, 640, CV_8UC1);
//		drawText(image);
//		imshow("Sample", image);
//		waitKey(0);
//	}
//	return 0;
//}

//void drawText(Mat & image)
//{
//	putText(image, "Hello OpenCV",
//		Point(20, 50),
//		FONT_HERSHEY_COMPLEX, 1, // font face and scale
//		Scalar(255, 255, 255), // white
//		1, LINE_AA); // line thickness and type
//}


//find_obstacle函数是获取深度图障碍物的函数,返回值是每个障碍物凸包的坐标，参数一depth是realsense返回的深度图（ushort型），
//参数二thresh和参数三max_thresh，是二值化的参数，参数四是凸包的最小有效面积，小于这个面积的障碍物可以视为噪点。
//函数首先筛选掉距离大于安全距离的点，然后进行阀值化和开运算减少一下噪点，用findContours得到轮廓图，最后用convexHull得到每个障碍物的凸包，最后返回坐标

//mask_depth函数是对深度图二值化，第一个参数image是原图，第二个参数th是目标图，第三个参数throld是最大距离，单位是mm，大于这个距离
//即为安全，不用考虑。


//定义了一个二值化函数，对深度图进行二值化
void mask_depth(Mat &image, Mat& th, int throld = 1000) {
	int nr = image.rows; // number of rows 
	int nc = image.cols; // number of columns 
	for (int i = 0; i < nr; i++) {
		for (int j = 0; j < nc; j++) {
			if (image.at<uchar>(i, j) > throld) //大于临界值的设为0
				th.at<uchar>(i, j) = 0;
		}
	}
}


void threechannel2gray(Mat &img, Mat &grayimg) {
	int row = img.rows;
	int col = img.cols;
	//Mat grayimg = Mat(row, col, CV_8UC1);//构造mat矩阵单通道用于存放灰度图

	//彩色图像访问，转换为灰度图
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < col; j++)
		{
			uchar b = img.at<Vec3b>(i, j)[0];//注意rgb的存放顺序
			uchar g = img.at<Vec3b>(i, j)[1];
			uchar r = img.at<Vec3b>(i, j)[2];
			//grayimg.at<uchar>(i, j) = r * 0.299 + g * 0.587 + b * 0.114;//公式
			grayimg.at<uchar>(i, j) = r * 1.000 + g * 0.000 + b * 0.000;//只提取红色通道的值，即距离最近的。
		}
	}
}

//定义了一个获取深度图障碍物的函数，返回一个vector
vector< vector<Point> > find_obstacle(Mat &depth, int thresh = 127, int max_thresh = 255, int area = 1000) {
	Mat dep;
	depth.copyTo(dep);  //CopyTo函数，将深度图depth复制到新的dep中去
	imshow("depth", depth);
	mask_depth(depth, dep, 255);//二值化，距离大于临界值的设为0，即黑色的；此处阈值为255，说明没用
	//imshow("dep1", dep);
	dep.convertTo(dep, CV_8UC3, 255);  //convertTo函数，将数据的类型进行了一些变化//参数调得越小，图像越暗
	//dep.convertTo(dep, CV_8UC1, 255);  //convertTo函数，将数据的类型进行了一些变化
	//imshow("dep2", dep);
	
	////进行开操作，去除小的物体（噪点），结 RPH_RECT, Size(15, 15));  //矩形核，核的大小可适当调整
	//Mat out;
	//morphologyEx(dep, out, MORPH_OPEN, element);
	////dilate(dhc, out, element);
	//imshow("opencv", out);//显示开操作效果图

	Mat src_copy = dep.clone();//复制
	Mat threshold_output;
	vector<vector<Point> > contours;//绘制矩形框，二维浮点型向量，将来会存储找到的边界的（x,y）坐标
	vector<Vec4i> hierarchy;//层次结构，等级；vector中放入4维int向量
	RNG rng(12345);//构造随机数，
	//对彩色图像进行二值化
	//threshold(dep, threshold_output, thresh, 255, THRESH_BINARY);//255是图像中的最大值，该函数直接对彩色图像进行了二值化，是允许的
	//imshow("threshold_output", threshold_output);
	//mask_depth(src, threshold_output);

// 寻找轮廓
	//三通道图转化为单通道灰度图
	vector<Mat> channels;//三通道图转化为单通道图
	split(dep, channels);
	Mat threshold_output_gray=Mat::zeros(dep.size(), CV_8UC1);//构造mat矩阵单通道用于存放灰度图;
	threechannel2gray(dep, threshold_output_gray);//目前只提取红色通道，即只识别近处物体
	imshow("threshold_output_red", threshold_output_gray);
	Mat threshold_output_binary;
	//转化成单通道灰度图后，对单通道灰度图进行二值化
	threshold(threshold_output_gray, threshold_output_binary, thresh, 255, THRESH_BINARY);
	//障碍物是白色的，带有黑色的噪点，因此进行闭操作，去除小的物体（噪点），结果输出给output
	Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));  //矩形核，核的大小可适当调整
	Mat medium;
	Mat output;
	morphologyEx(threshold_output_binary, medium, MORPH_CLOSE, element);
	morphologyEx(medium, output, MORPH_OPEN, element);
	imshow("output", output);//显示开闭操作效果图

	findContours(output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	//threshold_output是输入图像，
	//contours-输出轮廓的矩形框
	//RETR_TREE 检索所有的轮廓，并重构嵌套轮廓的整个层次
	//CHAIN_APPROX_SIMPLE：压缩水平的、垂直的和斜的部分，也就是，函数只保留他们的终点部分；CV.CHAIN_APPROX_SIMPLE - 压缩水平、垂直、对角线方向的元素，仅留该方向的终点坐标，例如一个矩形轮廓只需4个点来保存轮廓信息

	// 对每个轮廓计算其凸包
	vector<vector<Point> > hull(contours.size());    //定义了凸包的输出
	vector<vector<Point> > result;
	for (int i = 0; i < contours.size(); i++) {
		convexHull(Mat(contours[i]), hull[i], false);   //调用convexHull函数， 计算出图像的凸包，根据图像的轮廓点，通过函数convexhull转化成凸包的点点坐标，从而画出图像的凸包。
	}

	// 绘出轮廓及其凸包
	Mat drawing = Mat::zeros(output.size(), CV_8UC1);
	for (int i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) < area)//面积小于area的凸包，可忽略
			continue;
		result.push_back(hull[i]);
		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));//在0~255随机生成一个整数
		//Scalar color = Scalar(rng.uniform(0, 255));//在0~255随机生成一个整数
		drawContours(drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point());
		drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
	}
	imshow("contours", drawing);
	return result;
}

//int main(int argc, char* argv[]) {
//	Mat dhc;
//	Mat dep;
//	int idxImageRes = 1, idxFrameRate = 30;
//	RSWrapper depthCam(idxImageRes, idxImageRes, idxFrameRate, idxFrameRate);   //可能是intel库函数
//	// 四个参数分别是彩色相机的size索引，深度相机的size索引，彩色相机的帧率，深度相机的帧率
//	//Csize：424*240,640*480,1280*720；Dsize：480*270,640*480,1280*720；索引分别是0,1,2
//	if (!depthCam.init()) {
//		std::cerr << "Init. RealSense Failure!" << std::endl;
//		return -1;
//	}
//
//	while (true) {
//		//Get RGB-D Images
//		Mat color, depth;
//		bool ret = depthCam.capture(idxImageRes,color, depth);
//		//bool capture(int idx, cv::Mat &color, cv::Mat &depth);
//		if (!ret) {
//			std::cerr << "Get realsense camera data failure!" << std::endl;
//			break;
//		}
//
//		vector<vector<Point> > result;
//		result = find_obstacle(depth, 20, 255, 500);
//
//		if (waitKey(1) == 27) break;
//	}
//
//	depthCam.release();
//}

int main(int argc, char* argv[]) {
	// Create a Pipeline, which serves as a top-level API for streaming and processing frames
	pipeline p;

	// Configure and start the pipeline
	p.start();

	while (true) {
		// Block program until frames arrive
		frameset frames = p.wait_for_frames();
		
		//获取RGB图
		frame colorFrames = frames.get_color_frame();
		// 查询帧大小（宽度和高度）
		const int cw = colorFrames.as<video_frame>().get_width();
		const int ch = colorFrames.as<video_frame>().get_height();
		//帧转化为Mat
		Mat colorImage = Mat(Size(cw, ch), CV_8UC3, (void*)colorFrames.get_data());
		//d435 是RGB模式 而 cv是 BGR模式 ，所以交换一下
		cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);
		//imshow("colorImage", colorImage);
		

		//获取深度图
		frame depthframes = frames.get_depth_frame();
		//depth = get_depth_frame(frames, rs2_stream::depth_aligned_to_rectified_color);
		
		//为深度数据的可视化显示深度着色器
		static colorizer color_map;
		// 查询帧大小（宽度和高度）
		const int dw = depthframes.as<video_frame>().get_width();
		const int dh = depthframes.as<video_frame>().get_height();

		static frame color_depth_frames;
		color_depth_frames = color_map.colorize(depthframes);

		// 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
		Mat depthImage = Mat(Size(dw, dh), CV_8UC3, (void*)color_depth_frames.get_data());//8位三通道

		//imshow("depthImage", depthImage);

		////显示图片100ms，若没有下列语句，图像将不能保持
		//waitKey(100);


		//Mat depth = imread("C:/Users/zjsyzyt/Desktop/imagett.jpg", 0);
		// The frameset might not contain a depth frame, if so continue until it does
		//if (!depth) continue;

		//Mat dep;
		//depthImage.copyTo(dep);  //CopyTo函数，将深度图depth复制到新的dep中去
		vector<vector<Point> > result;
		result = find_obstacle(depthImage, 20, 255, 500);

		if (waitKey(100) == 27) break; //用户按下ESC(ASCII码为27)
	}

}
