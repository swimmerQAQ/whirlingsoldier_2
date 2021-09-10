
#pragma once
/**
* @file find.hpp
* @author
* @brief
旷绍宸 (1498507570@qq.com)
库类的定义
detect_F 父类找寻四个点
wirlsoldier 检测装甲板
shenFu 检测能量机关块
* @version 0.0
* @date 2021-09-10
*
* @copyright Copyright SCUT RobotLab(c) 2021
*
*/
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

/*   detect_F 的F 意思是 作为其他detection的父类使用，Father 同时F代表的是four points指代该类主要是完成对每个图像的预处理   */
class detect_F
{
public:
	
	detect_F();
	~detect_F();
    //画出四个角点
	int findExternal(Mat & in_img, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth);
    //找到中心店
    Point2f centrePoint(vector<Point> & in_contours);// import one contour return the centre point
    
};

//---------------------------------------------------在构造函数中就执行了detect_F父类的findexternal函数，为后面检测算出已知量---------------------------------------------------------//
class shenFu : public detect_F
{
    public:
    shenFu(Mat & origin, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth);
    ~shenFu();
    //检测能量机关
    void SSSFu(Mat & origin, vector<vector<Point>> & contours,int i, vector<Vec4i> & hierarchy,
                vector<float> & area ,vector<float> & external);
};
class whirlSoldier : public detect_F
{
    public:
    whirlSoldier(Mat & origin, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth);
    ~whirlSoldier();
    //检测装甲板
    void Soldier(Mat & origin ,vector<vector<Point>> & contours ,vector<float> & area, vector<float> & compartment, 
                    vector<vector<Point>> & P , vector<float> & height , vector<float> & wideth ,vector<float> & radius);
};