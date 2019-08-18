// test2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
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

int main()
{
	pipeline p;
	
	

	frameset frames;
	//realsenseD435 拍摄到的帧
	try {
		p.start();
		frames = p.wait_for_frames();
	}
	catch (Exception e) {
		cout << e.msg;
	}
	

	//获取RGB图
	frame colorFrames = frames.get_color_frame();
	// 查询帧大小（宽度和高度）
	const int cw = colorFrames.as<video_frame>().get_width();
	const int ch = colorFrames.as<video_frame>().get_height();
	//帧转化为Mat
	Mat colorImage = Mat(Size(cw, ch), CV_8UC3, (void*)colorFrames.get_data());
	//d435 是RGB模式 而 cv是 BGR模式 ，所以交换一下
	cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);
	imshow("colorImage", colorImage);

	frame depthFrames = frames.get_depth_frame();
	//为深度数据的可视化显示深度着色器
	static colorizer color_map;
	// 查询帧大小（宽度和高度）
	const int dw = depthFrames.as<video_frame>().get_width();
	const int dh = depthFrames.as<video_frame>().get_height();

	static frame color_depth_frames;
	color_depth_frames = color_map.colorize(depthFrames);

	// 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
	Mat depthImage = Mat(Size(dw, dh), CV_8UC3, (void*)color_depth_frames.get_data());
	imshow("depthImage", depthImage);

	//显示图片100ms，然后换下一帧
	waitKey(100);


}

