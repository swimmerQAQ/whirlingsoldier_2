/**
* @file main.hpp
* @author
* @brief
旷绍宸 (1498507570@qq.com)
库类定义
detect_F 父类找寻四个点
wirlsoldier 检测装甲板
shenFu 检测能量机关块

method:处理图像——画出每个图形的四个角点——对已知的点特征处理——检测画出目标

* @version 0.2
* @date 2021-09-10-09:46
*
* @copyright Copyright SCUT RobotLab(c) 2021
*
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include "../hsv_Color_pre/hsv_color_pre.hpp"
#include "../contours_tree/treeHierarchy.hpp"
#include "../find_centre/find.hpp"
#include "../kalmanFliter/kalman.hpp"
#include "../solvepnp/solvepnp.hpp"
#include "math.h"
using namespace std;
using namespace cv;
int hmin = 55;
int hmax = 159;
int smin = 98;
int smax = 255;
int vmin = 80;
int vmax = 226;
Mat imgmask,mask;
Mat gray,P;
Mat imgblur,imgcanny;
Mat imgdilute,imgerode;
vector<vector<Point>> contours;
vector<Vec4i> hierachy; 

/**
* @brief
detect_F作为父类被调用，获取外围四个角点，帮助检测
------------------
*
* @param contours  图像存储
* @param origin	   原始图像
* @param P		   四个角点
* @param hierarchy 图像顺序树
* @param radius	   四个角点延伸出来的圈住区域的圆半径
* @param height	   高度
* @param wideth	   宽度
* @param compartment 图像的间隔暂存
* @param external  最外围区域面积
* @param area      区域面积
*/

void detect(Mat origin, Mat in_image)
{
	findContours(in_image,contours,hierachy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	vector<vector<Point>> P(contours.size());
	vector<float> radius(contours.size());
	vector<float> height(contours.size());
	vector<float> wideth(contours.size());
	vector<float> compartment(contours.size());
	vector<float> area(contours.size());
	cout << "contours.size = " << contours.size() << endl;
	
		whirlSoldier test1(origin, contours ,P, radius ,radius, height, wideth);
		test1.Soldier(origin ,contours ,area, compartment, P , height , wideth , radius);
}
int main()
{
	VideoCapture frame;
	Mat image;
	string path = "../src/1.avi";
	image = frame.open(path);
	if(!frame.isOpened())
	{
		cout << "can not open the document.avi" << endl; 
	}
	hsv_color_pre();
	while (frame.read(image))
	{
		
		cv::cvtColor(image,imgmask,COLOR_BGR2HSV);//when i try to use self_class, the type become uncertain !!!???
		Scalar lower(hmin,smin,vmin);
		Scalar upper(hmax,smax,vmax);
		inRange(imgmask,lower,upper,mask);
		//imshow("cvt",mask);

		//----------------常规处理-------------------
		GaussianBlur(mask,imgblur,Size(3,3),3);
		Canny(imgblur,imgcanny,25,100);
		cv::imshow("imgcanny",imgcanny);
		cv::cvtColor(image,gray,COLOR_BGR2GRAY);

		//二值化-------------------
		cv::threshold(gray,P,150,255,THRESH_BINARY);
		cv::imshow("thresh",P);




		detect(image,P);//由于detection没有imshow，后面要在对应的打印的图像上接 imshow函数
		cv::imshow("origin",image);
		if(waitKey(30)==32)
		{
			if(waitKey()==27)
			{
				break;
			}
		}
	}

	return 0;
}
