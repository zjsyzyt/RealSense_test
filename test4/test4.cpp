// test4.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
//#include "RSWrapper.hpp" 
#include <iostream>
#include "rs.hpp"
#include "stdint.h"

using namespace cv;
using namespace std;
using namespace rs2;


int main(int argc, char* argv[]) {
	// Create a Pipeline, which serves as a top-level API for streaming and processing frames
	pipeline p;

	// Configure and start the pipeline
	p.start();

	while (true) {
		// Block program until frames arrive
		frameset frames = p.wait_for_frames();

		////获取RGB图
		//frame colorFrames = frames.get_color_frame();
		//// 查询帧大小（宽度和高度）
		//const int cw = colorFrames.as<video_frame>().get_width();
		//const int ch = colorFrames.as<video_frame>().get_height();
		////帧转化为Mat
		//Mat colorImage = Mat(Size(cw, ch), CV_8UC3, (void*)colorFrames.get_data());
		////d435 是RGB模式 而 cv是 BGR模式 ，所以交换一下
		//cvtColor(colorImage, colorImage, cv::COLOR_BGR2RGB);
		////imshow("colorImage", colorImage);

		//获取深度图
		depth_frame depthframes = frames.get_depth_frame();//depth value is stored in Float.
		if (!depthframes) continue;//若没有抓到深度帧，则不执行后续程序，继续下一个循环

		//const int dw = depthframes.as<video_frame>().get_width();
		//const int dh = depthframes.as<video_frame>().get_height();

		float dw = depthframes.get_width();
		float dh = depthframes.get_height();

		float d_center = depthframes.get_distance(dw/2, dh/2);
		if (!d_center) continue;
		Mat depthimg = Mat(Size(dw, dh), CV_16U);
		//ushort *dpixels = (ushort*).p[0];
		// 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
		unsigned short depvalue;//直接用int8_t或者uint8_t
		for (int x = 0; x < dw; x++)
			for (int y = 0; y < dh; y++)
			{
				float d = depthframes.get_distance(x, y);
				if (d > (float)1.000)
					depthimg.at<unsigned short>(y, x) = (uchar)0;
				else
					depvalue = (unsigned short)(d * 65535);
					depthimg.at<unsigned short>(y, x) = depvalue;
			}

		imshow("depthImage", depthimg);

		//显示图片100ms，若没有下列语句，图像将不能保持 
		waitKey(100);
	}

}