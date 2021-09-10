#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

void solvepnp_init(void);
void self_definition(void);
void pnpDetect( RotatedRect imageCornor);
void find_image_axis(Mat origin);
Eigen::Vector3f count(void);