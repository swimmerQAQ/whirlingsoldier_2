-------------------------------------------
new 
mkdir build
cd build
cmake ..
make
./bin/wsws
ok!
主要结构
：
——处理图像——调用父类detect_F 
——画出每个图形的四个角点
——检测特征筛选 对已知的点特征处理 
—— 检测画出目标
