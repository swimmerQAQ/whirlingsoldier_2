/**
* @file findCentre.cpp
* @author
* @brief
旷绍宸 (1498507570@qq.com)
库函数
detect_F 父类找寻四个点
wirlsoldier 检测装甲板
shenFu 检测能量机关块
* @version 0.0
* @date 2021-09-10
*
* @copyright Copyright SCUT RobotLab(c) 2021
*
*/
#include "find.hpp"
#include <opencv2/opencv.hpp>
using namespace cv;
#include <iostream>
using namespace std;
//--------------------初始父类 detect_F
detect_F::detect_F()
	{
		cout << "creat one detect,help to find static four points" << endl;
	}
detect_F::~detect_F()
	{
		cout << "one detection killed" << endl;
	}


int detect_F::findExternal(Mat & origin, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth)
{
	for (int i=0;i<contours.size();i++)
	{
		P[i] = contours[i];
		//遍历每个图形所有的点
		area[i] = contourArea(contours[i]);
		for (int j = 0; j < contours[i].size(); j++)//攫取其中之一的图形
		{
            //取最值，边缘角点
			if (P[i][0].y >= contours[i][j].y)
			{
				P[i][0].y = contours[i][j].y;
			}
			if(P[i][0].x >= contours[i][j].x)
			{
				P[i][0].x = contours[i][j].x;
			}
			
			if(contours[i].size() > 4)//计算保护，存在图像一开始没有检测到那么多点，防止危险操作
			{
				
				if (P[i][2].y < contours[i][j].y)
				{
					P[i][2].y = contours[i][j].y;
				}
				if(P[i][2].x < contours[i][j].x)
				{
					P[i][2].x = contours[i][j].x;
				}
				
			}
		}
		if(contours[i].size() >= 4)//计算保护，存在图像一开始没有检测到那么多点，防止危险操作
			{
				P[i][1].y = P[i][0].y;
				P[i][1].x = P[i][2].x;//右上


				P[i][3].y = P[i][2].y;
				P[i][3].x = P[i][0].x;//左下

				radius[i] = (P[i][3].y - P[i][0].y) / 2;
				height[i] = fabs(P[i][1].y - P[i][2].y);
				wideth[i] = fabs(P[i][0].x - P[i][1].x);

				circle(origin,P[i][0],2,Scalar(255,0,0),FILLED,8);//bule 左上 ------- 图像中点不同颜色对应的稳定位置
			
				circle(origin,P[i][1],2,Scalar(0,255,0),FILLED,8);//green 右上
			
				circle(origin,P[i][2],2,Scalar(0,0,255),FILLED,8);//red 右下
			
				circle(origin,P[i][3],2,Scalar(255,255,255),FILLED,8);//white 左下
			}
			else
			{
				cout << " error contours.size = "<< contours[i].size() <<" lower than 4 " << endl;//发现危险图像，数组可能超出位置，core dumped
				height[i] = 0;
				wideth[i] = 0;
				
			}
	}
	return 1;
}


Point2f detect_F::centrePoint(vector<Point> & in_contours)
{
	vector<Point> contours_Bulf;
	contours_Bulf = in_contours;
	Point2f centre;
	Moments contours_moments;
	contours_moments = moments(contours_Bulf);
	centre = Point(static_cast<float> (contours_moments.m10/contours_moments.m00),static_cast<float> (contours_moments.m01/contours_moments.m00));
	
	return centre;
}
shenFu::shenFu(Mat & origin, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth)
							{
								findExternal(origin,contours,P, radius ,area, height, wideth);
								cout << " detect shenFu ! " << endl;
							}
shenFu::~shenFu()
{
	cout << " stop detect shenFu ! " << endl;
}
void shenFu::SSSFu(Mat & origin, vector<vector<Point>> & contours,int i, vector<Vec4i> & hierarchy,
                vector<float> & area ,vector<float> & external_area)
				{
					area[i] = contourArea(contours[i]);//获取第i个图形的面积
					if(hierarchy[i][3] != -1)//要求后前子父中的 父级不存在的图形，读取面积
					{
						external_area[i] = contourArea(contours[hierarchy[i][3]]);
						if(area[i] > 500 && area[i] < 900)//---------------------------------------------------------区域面积条件 2
						{
							if(external_area[i] < 4500 && external_area[i] > 1500)//----------------------------父级区域面积条件 3
                    		{
                        		drawContours(origin,contours,i,Scalar(0,0,255),1,8,hierarchy);
                        		centrePoint(contours[i]);//---------------------------------------------------------库----返回中心点


								RotatedRect rect = minAreaRect(contours[i]);//--------------------------------------转动矩形
                        		Point2f rect_p[4];//可用于计算pnp和描绘旋转矩形
								rect.points(rect_p);
                        		for(int j=0;j<4;j++)
                        		{
                        		    line(origin,rect_p[j],rect_p[(j+1)%4],Scalar(100,255,50),2,8);
                        		}
							}
						}




					}
					
				}
//------------------------------------------------------------------------------------------------------------------------------------
whirlSoldier::whirlSoldier(Mat & origin, vector<vector<Point>> & contours, vector<vector<Point>> & P, 
							vector<float> & radius ,vector<float> & area,vector<float> & height,vector<float> & wideth)
{
	findExternal(origin,contours,P, radius ,area, height, wideth);
	cout << " detect soldier ! " << endl;
}
whirlSoldier::~whirlSoldier()
{
	cout << " stop detect soldier ! " << endl;
}
void whirlSoldier::Soldier(Mat & origin ,vector<vector<Point>> & contours ,vector<float> & area, vector<float> & compartment, 
                    vector<vector<Point>> & P , vector<float> & height , vector<float> & wideth ,vector<float> & radius)
					{
						for(int l=0;l<contours.size();l++)
						{
							if(area[l]>50)
							{
								for(int k=0;k<contours.size();k++)//注意find other contours 只能检测到i，后面的i的点还没有算出，出现core dumped
								{
									if(k != l)
									{			
										if(contours[k].size()>4)//保证点数数组已经装填，不要出现未初始化的P
										{
											compartment[k] =fabs( P[l][1].x + P[l][0].x - P[k][1].x - P[k][0].x )/2;// the length of conpartment
											cout << " the length of conpartment " << compartment[k] << endl;
											if(fabs(P[l][0].y - P[k][0].y)<100)//两条垂直相差不太大
											{
												if(compartment[k] >=(height[l]*2) && compartment[k]<3*height[l])//间隔不能太大
												{
													if(height[l]>5*wideth[l])
													{
														//drawContours(origin,contours,l,Scalar(0,0,255),2,8,hierachy);
														rectangle(origin,P[l][0],P[k][2],Scalar(255,0,255),3);
														circle(origin,Point(P[l][0].x,(P[l][0].y+P[l][2].y)/2),radius[l],Scalar(0,255,0),FILLED,8);
														circle(origin,Point(P[k][0].x,(P[k][0].y+P[k][2].y)/2),radius[k],Scalar(0,255,255),FILLED,8);
														circle(origin,Point((P[l][0].x+P[k][1].x)/2,(P[l][0].y+P[k][2].y)/2),5,Scalar(0,0,255),FILLED,8);

													}
									
												}
											}
										}
									}
								}
							}

						}
					}
/* 原型 */
//detect_F test;
	//test.findExternal(origin,contours,P,radius,area,height,wideth);
	/*
	for (int i=0;i<contours.size();i++)
	{
		P[i] = contours[i];
		//遍历每个图形所有的点
		area[i] = contourArea(contours[i]);
		for (int j = 0; j < contours[i].size(); j++)
		{
            //取最值
			if (P[i][0].y >= contours[i][j].y)
			{
				P[i][0].y = contours[i][j].y;
			}
			if(P[i][0].x >= contours[i][j].x)
			{
				P[i][0].x = contours[i][j].x;
			}
			
			if(contours[i].size() > 4)
			{
				
				if (P[i][2].y < contours[i][j].y)
				{
					P[i][2].y = contours[i][j].y;
				}
				if(P[i][2].x < contours[i][j].x)
				{
					P[i][2].x = contours[i][j].x;
				}
				
			}
		}
		if(contours[i].size() >= 4)
			{
				P[i][1].y = P[i][0].y;
				P[i][1].x = P[i][2].x;//右上


				P[i][3].y = P[i][2].y;
				P[i][3].x = P[i][0].x;//左下

				radius[i] = (P[i][3].y - P[i][0].y) / 2;
				height[i] = fabs(P[i][1].y - P[i][2].y);
				wideth[i] = fabs(P[i][0].x - P[i][1].x);

				circle(origin,P[i][0],2,Scalar(255,0,0),FILLED,8);//bule 左上
			
				circle(origin,P[i][1],2,Scalar(0,255,0),FILLED,8);//green 右上
			
				circle(origin,P[i][2],2,Scalar(0,0,255),FILLED,8);//red 右下
			
				circle(origin,P[i][3],2,Scalar(255,255,255),FILLED,8);//white 左下
			}
			else
			{
				cout << " error contours.size = "<< contours[i].size() <<" lower than 4 " << endl;
				height[i] = 0;
				wideth[i] = 0;
			}
	}
	*/
		/*		for(int l=0;l<contours.size();l++)
				{
					if(area[l]>50)
					{
					for(int k=0;k<contours.size();k++)//注意find other contours 只能检测到i，后面的i的点还没有算出，出现core dumped
					{
						if(k != l)
						{
							if(contours[k].size()>4)//保证点数数组已经装填，不要出现未初始化的P
							{
							conpartment[k] =fabs( P[l][1].x + P[l][0].x - P[k][1].x - P[k][0].x )/2;// the length of conpartment
							cout << " the length of conpartment " << conpartment[k] << endl;
							if(fabs(P[l][0].y - P[k][0].y)<70)//两条垂直相差不太大
							{
								if(conpartment[k]>=(height[l]*0.6) && conpartment[k]<3*height[l])
								{
									if(height[l]>5*wideth[l])
									{
										//drawContours(origin,contours,l,Scalar(0,0,255),2,8,hierachy);
										rectangle(origin,P[l][0],P[k][2],Scalar(255,0,255),3);
										circle(origin,Point(P[l][0].x,(P[l][0].y+P[l][2].y)/2),radius[l],Scalar(0,255,0),FILLED,8);
										circle(origin,Point(P[k][0].x,(P[k][0].y+P[k][2].y)/2),radius[k],Scalar(0,255,255),FILLED,8);
										circle(origin,Point((P[l][0].x+P[k][1].x)/2,(P[l][0].y+P[k][2].y)/2),5,Scalar(0,0,255),FILLED,8);

									}
									
								}
							}
							}
						}
					}
					}

				}
			
			//   危险操作 core default（dumped）
			*/