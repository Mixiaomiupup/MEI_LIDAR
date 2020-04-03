//雷达

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <pthread.h>
#include <cmath>
#include <math.h>
// #include <thread>
#include <ros/ros.h> 
#include <chrono>
#include <unistd.h>
#include <pthread.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "serial.h"
#include "kalman.h"
// #include "test.h"

#include "pcl_conversions/pcl_conversions.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <fstream>
#include <pcl/filters/project_inliers.h>
#include "boost/bind.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace std::literals::chrono_literals;
typedef pcl::PointXYZ PointT;
const double PI = 3.1415926;
fstream f1("/home/mi/catkin_ws/data0.txt", ios::out | ios::trunc);

unsigned char send_buf[14] = { 0 };     //发数数组
unsigned char recive_buf[1] = { 0 };
int fd;
int err;
char ss[] = "/dev/ttyUSB0"; //
//卡尔曼 X Y ANG
Kalman KalmanRobX;
Kalman KalmanRobY;
Kalman KalmanLidarX;
Kalman KalmanLidarY;
Kalman KalmanRobTheta;
Kalman KalmanLidarTheta;
int kalmanflag;
void send_data()
{

	int len = 0;

	for (int i = 0; i < 3; i++)
	{

		len = UART0_Send(fd, send_buf, 14);
		/*
		if(len > 0)
			printf(" %d time send %d data successful\n",i,len);
		else
			printf("send data failed!\n");
	   */
		if (len == 0)
			printf("send data failed!\n");

	}
	for (int j = 0; j < 14; j++)
	{
		send_buf[j] = 0;
	}
}
int recive_data(int& L_OR_R) {
	int len = 0;
	for (int i = 0; i < 3; i++)
	{

		len = UART0_Recv(fd, recive_buf, 1);

		 if (len != 0){
				len = 1;
		 }
		// 	printf("recive data failed!\n");

	}
	L_OR_R = recive_buf[0];
	////f1<<"buf "<<recive_buf[0]<<endl;
	recive_buf[0] = 0;
	return len;

}
//2015.967,4267.94,2660,5320
// int cylinder_type[3][5] = {
// 	5320,4268,2660,2016,//0 4号柱子 类型0
// 	4268,2660,2016,2016,//1 3号柱子 类型1
// 	2660,2660,2016,2016 //2号柱子   类型2
// };
double distance(double a1, double b1, double a2, double b2) {
	return sqrt(pow(a1 - b1, 2) + pow(a2 - b2, 2));
}

class cylinder_Node {
public:
	double x;
	double y;
	double height;
	int type;
	int* fourDis;
	int disIndex;

	cylinder_Node() {
		x = 100000;
		y = 100000;
		height = 0;
		type = -1;
		disIndex = 0;
		fourDis = new int[4];

	}
	cylinder_Node(double x1, double y1, double height1) {
		x = x1;
		y = y1;
		height = height1;
		type = -1;
		disIndex = 0;
		fourDis = new int[4];

	}
	void setXYH(double x1, double y1, double height1) {
		x = x1;
		y = y1;
		height = height1;
	}

};
class cylinder_list {//邻接表

public:
	cylinder_Node* CLDRs;
	int nodeIndex;
	cylinder_list() {
		CLDRs = new cylinder_Node[5];
		nodeIndex = 0;

	}
	void insert(double x, double y, double height) {
		double dis;
		if (nodeIndex < 5) {

			for (int i = 0; i < nodeIndex; i++) {
				dis = distance(CLDRs[i].x, x, CLDRs[i].y, y);
				int j = CLDRs[i].disIndex - 1; //第i个柱子插入排序
				while (j >= 0 && dis > CLDRs[i].fourDis[j]) {
					CLDRs[i].fourDis[j + 1] = CLDRs[i].fourDis[j];
					j--;
				}
				CLDRs[i].fourDis[j + 1] = dis;
				CLDRs[i].disIndex++;
				j = CLDRs[nodeIndex].disIndex - 1;
				while (j >= 0 && dis > CLDRs[nodeIndex].fourDis[j]) {//新柱子插入排序
					CLDRs[nodeIndex].fourDis[j + 1] = CLDRs[nodeIndex].fourDis[j];
					j--;
				}
				CLDRs[nodeIndex].fourDis[j + 1] = dis;
				CLDRs[nodeIndex].disIndex++;

			}
			CLDRs[nodeIndex].x = x;
			CLDRs[nodeIndex].y = y;
			CLDRs[nodeIndex].height = height;
			nodeIndex++;

		}


	}
	void setType() {
		if (nodeIndex >= 4) {
			for (int i = 0; i < nodeIndex; i++) {//根据最大的柱间距离来给柱子分类
				if (abs(CLDRs[i].fourDis[0] - 5320) < 150) {
					CLDRs[i].type = 0;
				}
				if (abs(CLDRs[i].fourDis[0] - 4268) < 150) {
					CLDRs[i].type = 1;
				}
				if (abs(CLDRs[i].fourDis[0] - 2660) < 150) {
					CLDRs[i].type = 2;
				}


			}
		}

	}
	void printList() {
		for (int i = 0; i < nodeIndex; i++) {
			for (int j = 0; j < CLDRs[i].disIndex; j++) {
				f1 << CLDRs[i].fourDis[j] << " ";
			}
			f1 << endl;
		}
	}



	void get_coordinate(double& lider_Xb, double& lider_Yb, double& thetab, double& rob_Xb, double& rob_Yb,double& lider_Xr, double& lider_Yr, double& thetar, double& rob_Xr, double& rob_Yr, double offset_X, double offset_Y) {
		int i;
		double sinb, cosb,sinr,cosr;
		double global_Xb, global_Yb,global_Xr, global_Yr;
		double L_mid_X = 0, L_mid_Y = 0;
		double R_mid_X = 0, R_mid_Y = 0;
		double X0 = 10000, Y0 = 10000, X1 = 10000, Y1 = 10000;
		setType();
		int type0 = 0, type1 = 0, type2 = 0;
		int tp0[2] = { 0,0 };
		int tp1[2] = { 0,0 };
		int tp2[2] = { 0,0 };
		for (i = 0; i < nodeIndex; i++) {//统计每种柱子的个数
			if (CLDRs[i].type == 0) {
				tp0[type0] = i;
				type0++;
			}
			if (CLDRs[i].type == 1) {
				tp1[type1] = i;
				type1++;
			}
			if (CLDRs[i].type == 2) {
				tp2[type2] = i;
				type2++;
			}
		}
		f1 << "type0 " << type0 << " type1 " << type1 << " type2 " << type2 << endl;
		printList();
		if (nodeIndex >= 4) {
			if ((type0 == 2 && type1 == 2 && type2 == 1) || (type0 == 2 && type1 == 1 && type2 == 1)) {//五个柱子都找好了,或者少１或３号柱子
				if (CLDRs[tp0[0]].x > CLDRs[tp0[1]].x) {//tp0[0]处为0号 tp0[1]处为4号
					int tmp = tp0[0];
					tp0[0] = tp0[1];
					tp0[1] = tmp;
				}

				if (CLDRs[tp2[0]].x >= 0) {//在０　２之间
					X0 = CLDRs[tp0[0]].x;//0 2号柱子
					Y0 = CLDRs[tp0[0]].y;
					X1 = CLDRs[tp2[0]].x;
					Y1 = CLDRs[tp2[0]].y;
					sinb = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
					cosb = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
					sinr = sinb;
					cosr = cosb;
					global_Xb = 3090;//全场坐标系中柱子的坐标
					global_Yb = -25;
					global_Xr = 3090;//全场坐标系中柱子的坐标
					global_Yr = 25;
					lider_Xb = global_Xb - X0 * cosb + Y0 * sinb;
					lider_Yb = global_Yb - X0 * sinb - Y0 * cosb;
					lider_Xr = global_Xr - X0 * cosr + Y0 * sinr;
					lider_Yr = global_Yr - X0 * sinr - Y0 * cosr;
					rob_Xb = global_Xb - (X0 - offset_X) * cosb + (Y0 - offset_Y) * sinb;
					rob_Yb = global_Yb - (X0 - offset_X) * sinb - (Y0 - offset_Y) * cosb;
					rob_Xr = global_Xr - (X0 - offset_X) * cosr + (Y0 - offset_Y) * sinr;
					rob_Yr = global_Yr - (X0 - offset_X) * sinr - (Y0 - offset_Y) * cosr;
					thetab = asin(sinb) * 180 / PI;
					thetar = asin(sinr) * 180 / PI;
					f1 << "0 2 4" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
				}
				else {//在２　４之间
					sinb = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
					cosb = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
					sinr = sinb;
					cosr = cosb;
					global_Xb = 5750;//全场坐标系中柱子的坐标
					global_Yb = -25;
					global_Xr = 5750;//全场坐标系中柱子的坐标
					global_Yr = 25;
					lider_Xb = global_Xb - X0 * cosb + Y0 * sinb;
					lider_Yb = global_Yb - X0 * sinb - Y0 * cosb;
					lider_Xr = global_Xr - X0 * cosr + Y0 * sinr;
					lider_Yr = global_Yr - X0 * sinr - Y0 * cosr;
					rob_Xb = global_Xb - (X0 - offset_X) * cosb + (Y0 - offset_Y) * sinb;
					rob_Yb = global_Yb - (X0 - offset_X) * sinb - (Y0 - offset_Y) * cosb;
					rob_Xr = global_Xr - (X0 - offset_X) * cosr + (Y0 - offset_Y) * sinr;
					rob_Yr = global_Yr - (X0 - offset_X) * sinr - (Y0 - offset_Y) * cosr;
					thetab = asin(sinb) * 180 / PI;
					thetar = asin(sinr) * 180 / PI;
					f1 << "2 4 0" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
				}
				


			}
			if (type0 == 0 && type1 == 2 && type2 == 2) {//一定为缺 0号 或 4号
				//分两组，根据1类和2类距离为2660分成两组连线垂直y轴的圆柱
				//ｔp1[0],tp2[0]为垂直的一组，tp1[1]和tp2[1]为一组垂直的
				if (abs(distance(CLDRs[tp2[0]].x, CLDRs[tp1[0]].x, CLDRs[tp2[0]].y, CLDRs[tp1[0]].y) - 2016) <= 150) {//如果距离为2016说明两柱子为斜的一组柱子，则需要交换ty1[0]，typ1[1]
					int tmp = tp1[0];//tp1[0]和tp2[0]为一组
					tp1[0] = tp1[1];//tp1[1]和tp2[1]为一组
					tp1[1] = tmp;
				}
					//蓝场取左边的两个柱子,红场取右边
					L_mid_X = (CLDRs[tp2[0]].x + CLDRs[tp1[0]].x) / 2;
					L_mid_Y = (CLDRs[tp2[0]].y + CLDRs[tp1[0]].y) / 2;
					//根据Y分左右,Y大的是左边的那组
					if (L_mid_Y > CLDRs[tp2[1]].y) {
						R_mid_X = (CLDRs[tp2[1]].x + CLDRs[tp1[1]].x) / 2;
						R_mid_Y = (CLDRs[tp2[1]].y + CLDRs[tp1[1]].y) / 2;
					}
					else {

						R_mid_X = L_mid_X;
						R_mid_Y = L_mid_Y;
						L_mid_X = (CLDRs[tp2[1]].x + CLDRs[tp1[1]].x) / 2;
						L_mid_Y = (CLDRs[tp2[1]].y + CLDRs[tp1[1]].y) / 2;
						int tmp = tp2[0];
						tp2[0] = tp2[1];//tp1[0] tp2[0]里面的数是左边的柱子的下标，
						tp2[1] = tmp;//tp1[1] tp2[1]里面的数是右边的柱子的下标
						tmp = tp1[0];
						tp1[0] = tp1[1];
						tp1[1] = tmp;
					}
					f1<< "左边 " << CLDRs[tp1[0]].x << " " << CLDRs[tp1[0]].y << " " << CLDRs[tp2[0]].x << " " << CLDRs[tp2[0]].y << endl;
					f1<< "右边 " << CLDRs[tp2[1]].x << " " << CLDRs[tp2[1]].y << " " << CLDRs[tp1[1]].x << " " << CLDRs[tp1[1]].y << endl;
					//根据X分上下，左边的组的X中值小于左边的2类柱子(2号柱子)的X则为下四个，否则为上四个
					if (L_mid_X < CLDRs[tp2[0]].x) {
						f1<< "蓝场缺４号" << endl;
						X0 = CLDRs[tp1[0]].x;//0 2号柱子
						Y0 = CLDRs[tp1[0]].y;
						X1 = CLDRs[tp2[0]].x;
						Y1 = CLDRs[tp2[0]].y;
						sinb = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						cosb = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						global_Xb = 3090;//全场坐标系中柱子的坐标
						global_Yb = -25;
						lider_Xb = global_Xb - X0 * cosb + Y0 * sinb;
						lider_Yb = global_Yb - X0 * sinb - Y0 * cosb;
						rob_Xb = global_Xb - (X0 - offset_X) * cosb + (Y0 - offset_Y) * sinb;
						rob_Yb = global_Yb - (X0 - offset_X) * sinb - (Y0 - offset_Y) * cosb;
						thetab = asin(sinb) * 180 / PI;//sin有正负，用sin
						f1 << "0 2 0" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
					
					}
					else {
						f1<< "蓝场缺0号" << endl;
						X0 = CLDRs[tp2[0]].x;//2 4号柱子
						Y0 = CLDRs[tp2[0]].y;
						X1 = CLDRs[tp1[0]].x;
						Y1 = CLDRs[tp1[0]].y;
						sinb = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						cosb = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						global_Xb = 5750;//全场坐标系中柱子的坐标
						global_Yb = -25;
						lider_Xb = global_Xb - X0 * cosb + Y0 * sinb;
						lider_Yb = global_Yb - X0 * sinb - Y0 * cosb;
						rob_Xb = global_Xb - (X0 - offset_X) * cosb + (Y0 - offset_Y) * sinb;
						rob_Yb = global_Yb - (X0 - offset_X) * sinb - (Y0 - offset_Y) * cosb;
						thetab = asin(sinb) * 180 / PI;
						f1 << "2 4 2" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
		
					}
					//根据X分上下，左边的组的X中值小于左边的2类柱子(2号柱子)的X则为下四个，否则为上四个
					if (R_mid_X < CLDRs[tp2[0]].x) {
						f1<< "红场缺４号" << endl;
						X0 = CLDRs[tp1[1]].x;//0 2号柱子
						Y0 = CLDRs[tp1[1]].y;
						X1 = CLDRs[tp2[1]].x;
						Y1 = CLDRs[tp2[1]].y;
						sinr = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						cosr = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						global_Xr = 3090;//全场坐标系中柱子的坐标
						global_Yr = 25;
						lider_Xr = global_Xr - X0 * cosr + Y0 * sinr;
						lider_Yr = global_Yr - X0 * sinr - Y0 * cosr;
						rob_Xr = global_Xr - (X0 - offset_X) * cosr + (Y0 - offset_Y) * sinr;
						rob_Yr = global_Yr - (X0 - offset_X) * sinr - (Y0 - offset_Y) * cosr;
						thetar = asin(sinr) * 180 / PI;//sin有正负，用sin

						f1 << "0 2 0" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
					}
					else {
						f1<< "红场缺0号" << endl;
						X0 = CLDRs[tp2[1]].x;//2 4号柱子
						Y0 = CLDRs[tp2[1]].y;
						X1 = CLDRs[tp1[1]].x;
						Y1 = CLDRs[tp1[1]].y;
						sinr = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						cosr = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
						global_Xr = 5750;//全场坐标系中柱子的坐标
						global_Yr = 25;
						lider_Xr = global_Xr - X0 * cosr + Y0 * sinr;
						lider_Yr = global_Yr - X0 * sinr - Y0 * cosr;
						rob_Xr = global_Xr - (X0 - offset_X) * cosr + (Y0 - offset_Y) * sinr;
						rob_Yr = global_Yr - (X0 - offset_X) * sinr - (Y0 - offset_Y) * cosr;
						thetar = asin(sinr) * 180 / PI;
						f1 << "2 4 2" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
					}
				
			}
			// if (type0 == 2 && type1 == 2 && type2 == 0) {//缺２号柱子
			// 	//f1<< "缺２号柱子" << endl;
			// 	if (CLDRs[tp0[0]].x > CLDRs[tp0[1]].x) {//０号在tp0[0],4号在tp0[1]
			// 		int tmp = tp0[0];
			// 		tp0[0] = tp0[1];
			// 		tp0[1] = tmp;
			// 	}
			// 	X0 = CLDRs[tp0[0]].x;
			// 	Y0 = CLDRs[tp0[0]].y;
			// 	X1 = CLDRs[tp0[1]].x;
			// 	Y1 = CLDRs[tp0[1]].y;
			// 	sin = (Y0 - Y1) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));
			// 	cos = (X1 - X0) / sqrt(pow(X0 - X1, 2) + pow(Y0 - Y1, 2));

			// 		global_X = 3090;//全场坐标系中柱子的坐标
			// 		global_Y = -25;
			// 		global_X = 3090;//全场坐标系中柱子的坐标
			// 		global_Y = 25;


			// 	lider_X = global_X - X0 * cos + Y0 * sin;
			// 	lider_Y = global_Y - X0 * sin - Y0 * cos;
			// 	rob_X = global_X - (X0 - offset_X) * cos + (Y0 - offset_Y) * sin;
			// 	rob_Y = global_Y - (X0 - offset_X) * sin - (Y0 - offset_Y) * cos;
			// 	theta = asin(sin) * 180 / PI;
			// 	f1 << "0 4 2" << " X0: " << X0 << " Y0: " << Y0 << " X1 " << X1 << " Y1 " << Y1 << endl;
			// 	//f1<<"0 4 2";

			// }
		}



	}
};

//切割点云－－－－－－－－－－－－－－－－－－－
pcl::PointCloud<PointT>::Ptr partition(pcl::PointCloud<PointT>::Ptr clouds, double x0, double x1, double y0, double y1, double z0, double z1) {
	pcl::PointCloud<PointT>::Ptr cloud0(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(clouds);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(x0, x1);
	pass.filter(*cloud0);

	pass.setInputCloud(cloud0);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y0, y1);
	pass.filter(*cloud1);

	pass.setInputCloud(cloud1);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(z0, z1);
	pass.filter(*cloud2);
	return cloud2;
}
//切割圆柱－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－－
int segment(pcl::PointCloud<PointT>::Ptr cloud, double xl, double xr, double yl, double yr, double zl, double zr, double hl, double hr,cylinder_list* cylinders, double& X, double& Y, double& Height, pcl::PointCloud<PointT>::Ptr cloud_filtered1) {
	f1<< "xl " << xl << " xr " << xr << " yl " << yl << " yr " << yr << endl;
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	double sum_x = 0;
	double sum_y = 0;
	double x;
	double y;
	double X0, X1, Y0, Y1;
	double max_z = 0;
	double min_z = 0;
	double z = 0;
	double height;
	double dis;
	cloud_filtered = partition(cloud, xl, xr, yl, yr, zl, zr);
	if (cloud_filtered->points.size() == 0) {//点云里没有点的情况----------------------------
		f1<< "点云内没点，舍弃" << endl;
		return 0;
	}

	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud_filtered);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.01);//越小对远处的圆柱拟合越好,0.05容易把一个柱子切两次
	seg.setMaxIterations(10000);
	seg.setDistanceThreshold(0.1);//0.1基本不需要动
	seg.setRadiusLimits(-0.07, 0.07);
	seg.setInputCloud(cloud_filtered);
	seg.setInputNormals(cloud_normals);
	seg.segment(*inliers_cylinder, *coefficients_cylinder);

	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);

	extract.filter(*cloud_cylinder);
	if (cloud_cylinder->points.size() == 0) {//点云有点但是没有圆柱----------------------------
		f1<< "圆柱内没点，舍弃" << endl;
		return 1;
	}


	for (int i = 0; i < cloud_cylinder->points.size(); ++i) {
		sum_x += (double)cloud_cylinder->points[i].x * 1000;
		sum_y += (double)cloud_cylinder->points[i].y * 1000;


	}

	x = sum_x / cloud_cylinder->points.size();
	y = sum_y / cloud_cylinder->points.size();
	X0 = x;
	Y0 = y;
	sum_x = 0;
	sum_y = 0;

	cloud_filtered = partition(cloud, X0 / 1000 - 0.2, X0 / 1000 + 0.2, Y0 / 1000 - 0.2, Y0 / 1000 + 0.2, zl, zr);//在0.4的正方形内找
	if (cloud_filtered->points.size() > 0) {
		max_z = (double)cloud_filtered->points[0].z * 1000;
		min_z = (double)cloud_filtered->points[0].z * 1000;
	}
	for (int it = 0; it < cloud_filtered->points.size(); ++it) {

		sum_x += (double)cloud_filtered->points[it].x * 1000;
		sum_y += (double)cloud_filtered->points[it].y * 1000;
		z = (double)cloud_filtered->points[it].z * 1000;
		if (z > max_z) {
			max_z = z;
		}
		if (z < min_z) {
			min_z = z;
		}

	}
	height = max_z - min_z;
	//f1<<"height "<<height<<endl;
	if (height < hl*1000 ) {//将高度小于100的非圆柱排除，可能是墙的一线或两线
		f1<< "太矮了，舍弃" << endl;
		return 3;
	}
	if (height > hr*1000 ) {
		f1<< "太高了，舍去" << endl;
		return 3;
	}
	x = sum_x / cloud_filtered->points.size();
	y = sum_y / cloud_filtered->points.size();
	X1 = x;
	Y1 = y;

	if (distance(X0, X1, Y0, Y1) < 50) {//两次拟合的圆柱的距离小于50
		bool flag = true;
		for (int i = 0; i < cylinders->nodeIndex; i++) {//和之前找到的柱子每一个都算一次距离，所有距离都是四种中的一种才要这个柱子
			dis = distance(cylinders->CLDRs[i].x, X1, cylinders->CLDRs[i].y, Y1);
			if (abs(dis - 2016) < 150 || abs(dis - 2660) < 150 || abs(dis - 4268) < 150 || abs(dis - 5320) < 150) {
				flag = true;

			}
			else {
				flag = false;
				break;
			}
		}
		if (!flag) {
			f1<<"距离不满足要求舍去"<<X1<<" "<<Y1<<endl;

			return 3;
		}
		*cloud_filtered1 = *cloud_filtered;//把找到的柱子存起来
		X = X1;
		Y = Y1;
		Height = height;
		f1<< "满足要求找到柱子:"<< "X " << X << " Y " << Y << " Height " << Height << endl;
		return 2;




	}

	else {//找到的圆柱不对--------------------------------
	//f1<< "找到的柱子和点云不重叠" << endl;
		return 3;
	}
}

int segment_cylinder(pcl::PointCloud<PointT>::Ptr cloud, double xl, double xr, double yl, double yr, double zl, double zr, double hl, double hr, cylinder_list* cylinders, int& i, double& X, double& Y, double& Height, double& XF, double& YF, vector<pcl::PointCloud<PointT>::Ptr>& cloud_cylinders, int direction,  int next) {
	double mid = (yl + yr) / 2;
	int j0, j1;
	double X0 = 10000;
	double X1 = 10000;
	double Y0 = 10000;
	double Y1 = 10000;
	double height0, height1;
	pcl::PointCloud<PointT>::Ptr cloud_filtered0(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered1(new pcl::PointCloud<PointT>);
	if (next == 0) {//如果是找下两个，xl,xr应该已经前推或者后递，不需要把当前的柱子的y方向左右各切割0.2，这样做是为了找到Y和当前柱子相同的下两个柱子
		//f1<<"右边"<<endl;
		j0 = segment(cloud, xl, xr, yl, mid, zl, zr, hl, hr,cylinders, X0, Y0, height0, cloud_filtered0);
		//f1<<"左边"<<endl;
		j1 = segment(cloud, xl, xr, mid, yr, zl, zr, hl, hr,cylinders, X1, Y1, height1, cloud_filtered1);
	}
	else {//如果是找下一个，需要把当前柱子的y方向左右各切割0.2，因为此时x没有往前推，这样做是希望找到和当前柱子Ｘ相同的柱子
		//f1<<"右边"<<endl;
		j0 = segment(cloud, xl, xr, yl, mid - 0.2, zl, zr, hl, hr,cylinders, X0, Y0, height0, cloud_filtered0);
		//f1<<"左边"<<endl;
		j1 = segment(cloud, xl, xr, mid + 0.2, yr, zl, zr, hl, hr,cylinders, X1, Y1, height1, cloud_filtered1);

	}


	if (j0 == 2 && j1 != 2 && X0 != 10000 && Y0 != 10000 && i <= 4) {

		X = X0;
		Y = Y0;
		Height = height0;
		*cloud_cylinders[i] = *cloud_filtered0;//把找到的柱子存起来
		if (i == 0) {
			XF = X;
			YF = Y;

		}
		cylinders->insert(X0, Y0, height0);

		i += 1;
		//f1<<"i= "<<i<<endl;
		return 2;
	}
	else if (j1 == 2 && j0 != 2 && X1 != 10000 && Y1 != 10000 && i <= 4) {
		X = X1;
		Y = Y1;
		Height = height1;
		*cloud_cylinders[i] = *cloud_filtered1;//把找到的柱子存起来
		if (i == 0) {
			XF = X;
			YF = Y;
		}
		cylinders->insert(X1, Y1, height1);

		i += 1;
		//f1<<"i= "<<i<<endl;
		return 2;
	}
	else if (j0 == 2 && j1 == 2 && X0 != 10000 && Y0 != 10000 && distance(X0, X1, Y0, Y1) < 50 && i <= 4) {
		if (cloud_filtered0->points.size() > cloud_filtered1->points.size()) {
			X = X0;
			Y = Y0;
			Height = height0;
			*cloud_cylinders[i] = *cloud_filtered0;
			if (i == 0) {
				XF = X;
				YF = Y;

			}
			cylinders->insert(X0, Y0, height0);
			i += 1;
			//f1<<"i= "<<i<<endl;
			return 2;
		}
		else {
			X = X1;
			Y = Y1;
			Height = height1;
			*cloud_cylinders[i] = *cloud_filtered1;//把找到的柱子存起来
			if (i == 0) {
				XF = X;
				YF = Y;

			}
			cylinders->insert(X1, Y1, height1);
			i += 1;
			//f1<<"i= "<<i<<endl;
			return 2;
		}
	}
	else if (j0 == 2 && j1 == 2 && X0 != 10000 && Y0 != 10000 && X1 != 10000 && Y1 != 10000 && distance(X0, X1, Y0, Y1) > 50 && i <= 3) {//还有两个位置

		if (direction) {//向后
			if (X0 > X1) {
				X = X1;//选小
				Y = Y1;
				Height = height1;
				*cloud_cylinders[i] = *cloud_filtered0;//存近的
				if (i == 0) {//选大
					XF = X0;
					YF = Y0;
				}
				cylinders->insert(X0, Y0, height0);
				*cloud_cylinders[i + 1] = *cloud_filtered1;
				cylinders->insert(X1, Y1, height1);
			}
			else {
				X = X0;//选小
				Y = Y0;
				Height = height0;
				*cloud_cylinders[i] = *cloud_filtered1;
				if (i == 0) {//选大
					XF = X1;
					YF = Y1;

				}
				cylinders->insert(X1, Y1, height1);
				*cloud_cylinders[i + 1] = *cloud_filtered0;
				cylinders->insert(X0, Y0, height0);
			}
		}
		else {//向前

			if (X0 > X1) {
				X = X0;//选大
				Y = Y0;
				Height = height0;
				*cloud_cylinders[i] = *cloud_filtered1;//存近的
				if (i == 0) {//选小
					XF = X1;
					YF = Y1;

				}
				cylinders->insert(X1, Y1, height1);
				*cloud_cylinders[i + 1] = *cloud_filtered0;
				cylinders->insert(X0, Y0, height0);
			}
			else {
				X = X1;//选大
				Y = Y1;
				Height = height1;
				*cloud_cylinders[i] = *cloud_filtered0;
				if (i == 0) {//选小
					XF = X0;
					YF = Y0;

				}

				cylinders->insert(X0, Y0, height0);
				*cloud_cylinders[i + 1] = *cloud_filtered1;
				cylinders->insert(X1, Y1, height1);
			}
		}

		i += 2;//加2
		//f1<<"i= "<<i<<endl;
		return 2;

	}
	else {
		return 0;
	}

}
void quickSort(pcl::PointCloud<PointT>::Ptr cloud_filtered1, int low, int high) {
	if (low >= high) {
		return;
	}
	int i = low;
	int j = high;
	double pivotx = cloud_filtered1->points[low].x;
	double pivoty = cloud_filtered1->points[low].y;
	double pivotz = cloud_filtered1->points[low].z;

	while (i < j) {
		while (i < j && pivotz >= cloud_filtered1->points[j].z) {
			j--;
		}
		if (i < j) {
			cloud_filtered1->points[i].x = cloud_filtered1->points[j].x;
			cloud_filtered1->points[i].y = cloud_filtered1->points[j].y;
			cloud_filtered1->points[i].z = cloud_filtered1->points[j].z;
		}
		while (i < j && pivotz <= cloud_filtered1->points[i].z) {
			i++;
		}
		if (i < j) {
			cloud_filtered1->points[j].x = cloud_filtered1->points[i].x;
			cloud_filtered1->points[j].y = cloud_filtered1->points[i].y;
			cloud_filtered1->points[j].z = cloud_filtered1->points[i].z;
		}

	}
	cloud_filtered1->points[i].x = pivotx;
	cloud_filtered1->points[i].y = pivoty;
	cloud_filtered1->points[i].z = pivotz;
	quickSort(cloud_filtered1, low, i - 1);
	quickSort(cloud_filtered1, i + 1, high);
}
int segment_post(pcl::PointCloud<PointT>::Ptr cloud, double xl, double xr, double yl, double yr, double zl, double zr, double& X, double& Y, double& Height, pcl::PointCloud<PointT>::Ptr cloud_filtered1) {
	//f1<< "xl " << xl << " xr " << xr << " yl " << yl << " yr " << yr << endl;
	pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr  inliers_cylinder(new pcl::PointIndices);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
	double sum_x = 0;
	double sum_y = 0;
	double x;
	double y;
	double max_z = 0;
	double min_z = 0;
	double z = 0;
	double height = 0;
	cloud_filtered = partition(cloud, xl, xr, yl, yr, zl, zr);
	//////f1<< "cloud_filtered " << cloud_filtered->points.size() << endl;
	if (cloud_filtered->points.size() == 0) {//点云里没有点的情况----------------------------
		//////f1<< "点云内没点" << endl;
		return 0;
	}
	quickSort(cloud_filtered, 0, cloud_filtered->points.size() - 1);

	if (cloud_filtered->points.size() > 0) {
		max_z = cloud_filtered->points[0].z;
		min_z = cloud_filtered->points[0].z;
	}
	int layer = 0;
	int first = 0;
	int cnt = 0;
	for (int i = 0; i < cloud_filtered->points.size() && layer < 3; ) {
		z = cloud_filtered->points[i].z;
		if (abs(z - cloud_filtered->points[first].z) < 0.1) {
			sum_x += (double)cloud_filtered->points[i].x * 1000;
			sum_y += (double)cloud_filtered->points[i].y * 1000;
			//f1<< i <<" layer "<<layer<<" x " << (double)cloud_filtered->points[i].x * 1000 << " y " << (double)cloud_filtered->points[i].y * 1000 << " z " <<cloud_filtered->points[i].z * 1000<< endl;
			i++;
			cnt++;

		}
		else {
			first = i;
			layer++;
		}
		if (z > max_z) {
			max_z = z;
		}
		if (z < min_z) {
			min_z = z;
		}
	}

	//f1<<"cnt="<<cnt<<endl;
	x = sum_x / cnt;
	y = sum_y / cnt;

	*cloud_filtered1 = *cloud_filtered;//把找到的柱子存起来
	X = x;
	Y = y;
	Height = height;
	return 2;//成功切割圆柱---------------------

}
void get_Post(pcl::PointCloud<PointT>::Ptr cloud, double lider_Xb, double lider_Yb, double thetab,double lider_Xr, double lider_Yr, double thetar ,vector<pcl::PointCloud<PointT>::Ptr>& cloud_cylinders, double& post_thetab,double& post_thetar) {
	//f1<<"lider_X "<<lider_X<<" lider_Y "<<lider_Y<<" theta "<<theta<<" i "<<i<<endl;
	double sinb = sin(thetab * PI / 180);
	double cosb = cos(thetab * PI / 180);
	double sinr = sin(thetab * PI / 180);
	double cosr = cos(thetab * PI / 180);
	//	double tan1 = tan(theta*PI/180);
	double global_right_post_Xb = 10164.5;
	double global_right_post_Yb = 2005.5;
	double global_left_post_Xb = 10164.5;
	double global_left_post_Yb = 3094.5;
	double global_mid_post_Xb = 10164.5;
	double global_mid_post_Yb = 2550;

	double global_right_post_Xr = 10164.5;
	double global_right_post_Yr = -3094.5;
	double global_left_post_Xr = 10164.5;
	double global_left_post_Yr = -2005.5;
	double global_mid_post_Xr = 10164.5;
	double global_mid_post_Yr = -2550;



	double right_post_Xb = (global_right_post_Xb - lider_Xb) * cosb + (global_right_post_Yb - lider_Yb) * sinb;
	double right_post_Yb = (global_right_post_Xb - lider_Xb) * (-sinb) + (global_right_post_Yb - lider_Yb) * cosb;
	double left_post_Xb = (global_left_post_Xb - lider_Xb) * cosb + (global_left_post_Yb - lider_Yb) * sinb;
	double left_post_Yb = (global_left_post_Xb - lider_Xb) * (-sinb) + (global_left_post_Yb - lider_Yb) * cosb;
	double mid_post_Xb = (global_mid_post_Xb - lider_Xb) * cosb + (global_mid_post_Yb - lider_Yb) * sinb;
	double mid_post_Yb = (global_mid_post_Xb - lider_Xb) * (-sinb) + (global_mid_post_Yb - lider_Yb) * cosb;
	
	double right_post_Xr = (global_right_post_Xr - lider_Xr) * cosr + (global_right_post_Yr - lider_Yr) * sinr;
	double right_post_Yr = (global_right_post_Xr - lider_Xr) * (-sinr) + (global_right_post_Yr - lider_Yr) * cosr;
	double left_post_Xr = (global_left_post_Xr - lider_Xr) * cosr + (global_left_post_Yr - lider_Yr) * sinr;
	double left_post_Yr = (global_left_post_Xr - lider_Xr) * (-sinr) + (global_left_post_Yr - lider_Yr) * cosr;
	double mid_post_Xr = (global_mid_post_Xr - lider_Xr) * cosr + (global_mid_post_Yr - lider_Yr) * sinr;
	double mid_post_Yr = (global_mid_post_Xr - lider_Xr) * (-sinr) + (global_mid_post_Yr - lider_Yr) * cosr;

	pcl::PointCloud<PointT>::Ptr cloud_filtered0(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered1(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered3(new pcl::PointCloud<PointT>);
	//f1<<"X0 "<<x0<<" Y0 "<<y0<<" X1 "<<x1<<" Y1 "<<y1<<endl;
	double real_right_post_Xb = 0;
	double real_right_post_Yb = 0;
	double height0b = 0;
	double real_left_post_Xb = 0;
	double real_left_post_Yb = 0;
	double height1b = 0;
	int j0b, j1b;
	//Ｚ的高度需要调整，２可以保证不扫房顶
	j0b = segment_post(cloud, right_post_Xb / 1000 - 0.3, right_post_Xb / 1000 + 0.3, right_post_Yb / 1000 - 0.3, right_post_Yb / 1000 + 0.3, 0.2, 2, real_right_post_Xb, real_right_post_Yb, height0b, cloud_filtered0);
	j1b = segment_post(cloud, left_post_Xb / 1000 - 0.3, left_post_Xb / 1000 + 0.3, left_post_Yb / 1000 - 0.3, left_post_Yb / 1000 + 0.3, 0.2, 2, real_left_post_Xb, real_left_post_Yb, height1b, cloud_filtered1);

	double real_right_post_Xr = 0;
	double real_right_post_Yr = 0;
	double height0r = 0;
	double real_left_post_Xr = 0;
	double real_left_post_Yr = 0;
	double height1r = 0;
	int j0r, j1r;
	//Ｚ的高度需要调整，２可以保证不扫房顶
	j0r = segment_post(cloud, right_post_Xr / 1000 - 0.3, right_post_Xr / 1000 + 0.3, right_post_Yr / 1000 - 0.3, right_post_Yr / 1000 + 0.3, 0.2, 2, real_right_post_Xr, real_right_post_Yr, height0r, cloud_filtered2);
	j1r = segment_post(cloud, left_post_Xr / 1000 - 0.3, left_post_Xr / 1000 + 0.3, left_post_Yr / 1000 - 0.3, left_post_Yr / 1000 + 0.3, 0.2, 2, real_left_post_Xr, real_left_post_Yr, height1r, cloud_filtered3);


	if (real_right_post_Xb != 0 && real_right_post_Yb != 0) {

		//f1<< "real_right_post_X " << real_right_post_X << " real_right_post_Y " << real_right_post_Y << endl;
		//f1<< distance(real_right_post_X, right_post_X, real_right_post_Y, right_post_Y) << endl;
	}
	if (real_left_post_Xb != 0 && real_left_post_Yb != 0) {

		//f1<< " real_left_post_X " << real_left_post_X << " real_left_post_Y " << real_left_post_Y << endl;
		//f1<< distance(real_left_post_X,left_post_X, real_left_post_Y,left_post_Y) << endl;;
	}

	double mid_xb, mid_yb;
	double post_disb = distance(real_right_post_Xb, real_left_post_Xb, real_right_post_Yb, real_left_post_Yb);
	//f1<<post_dis<<endl;
	mid_xb = (real_right_post_Xb + real_left_post_Xb) / 2;
	mid_yb = (real_right_post_Yb + real_left_post_Yb) / 2;
	double real_sinb, count_sinb, real_thetab = 0, count_thetab = 0;
	count_sinb = mid_post_Yb / sqrt(pow(mid_post_Yb - 0, 2) + pow(mid_post_Xb - 0, 2));//算出来的		
	count_thetab = asin(count_sinb);
	*cloud_cylinders[5] = *cloud_filtered0;
	*cloud_cylinders[6] = *cloud_filtered1;
	if (abs(post_disb - 1089) < 150) {
		real_sinb = mid_yb / sqrt(pow(mid_yb - 0, 2) + pow(mid_xb - 0, 2));//测出来的
		real_thetab = asin(real_sinb);
		*cloud_cylinders[5] = *cloud_filtered0;
		*cloud_cylinders[6] = *cloud_filtered1;
	}
	if (real_thetab == 0) {
		post_thetab = count_thetab * 180 / PI;
	}
	else {
		post_thetab = real_thetab * 180 / PI;
	}

	double mid_xr, mid_yr;
	double post_disr = distance(real_right_post_Xr, real_left_post_Xr, real_right_post_Yr, real_left_post_Yr);
	//f1<<post_dis<<endl;
	mid_xr = (real_right_post_Xr + real_left_post_Xr) / 2;
	mid_yr = (real_right_post_Yr + real_left_post_Yr) / 2;
	double real_sinr, count_sinr, real_thetar = 0, count_thetar = 0;
	count_sinr = mid_post_Yr / sqrt(pow(mid_post_Yr - 0, 2) + pow(mid_post_Xr - 0, 2));//算出来的		
	count_thetar = asin(count_sinr);
	*cloud_cylinders[7] = *cloud_filtered2;
	*cloud_cylinders[8] = *cloud_filtered3;
	if (abs(post_disr - 1089) < 150) {
		real_sinr = mid_yr / sqrt(pow(mid_yr - 0, 2) + pow(mid_xr - 0, 2));//测出来的
		real_thetar = asin(real_sinr);
		*cloud_cylinders[7] = *cloud_filtered2;
		*cloud_cylinders[8] = *cloud_filtered3;
	}
	if (real_thetar == 0) {
		post_thetar = count_thetar * 180 / PI;
	}
	else {
		post_thetar = real_thetar * 180 / PI;
	}
}

void velodyneViewerCallback(const sensor_msgs::PointCloud2::ConstPtr& lidarMsg,int flag) {
	int i = 0;
	int j = 0;
	// All the objects needed
	pcl::PCDReader reader;

	pcl::PCDWriter writer;


	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);




	pcl::PointCloud<PointT>::Ptr cloud_cylinder0(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder1(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder2(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder3(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder4(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder5(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder6(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder7(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder8(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder9(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_cylinder10(new pcl::PointCloud<PointT>());
	vector<pcl::PointCloud<PointT>::Ptr> cloud_cylinders;
	cloud_cylinders.push_back(cloud_cylinder0);
	cloud_cylinders.push_back(cloud_cylinder1);
	cloud_cylinders.push_back(cloud_cylinder2);
	cloud_cylinders.push_back(cloud_cylinder3);
	cloud_cylinders.push_back(cloud_cylinder4);
	cloud_cylinders.push_back(cloud_cylinder5);
	cloud_cylinders.push_back(cloud_cylinder6);
	cloud_cylinders.push_back(cloud_cylinder7);
	cloud_cylinders.push_back(cloud_cylinder8);
	cloud_cylinders.push_back(cloud_cylinder9);
	cloud_cylinders.push_back(cloud_cylinder10);
	fromROSMsg(*lidarMsg, *cloud);
	int direction = 0;//前后
	double X, Y, Height;
	double lider_Xb = 10000,lider_Xr = 10000;//雷达坐标默认值
	double lider_Yb = 10000,lider_Yr = 10000;
	double rob_Xb = 10000,rob_Xr = 10000;//车坐标默认值
	double rob_Yb = 10000,rob_Yr = 10000;
	double offset_X = -165;//车心偏移值   -165
	double offset_Y = -251;//车心偏移值
	double XF = 0;
	double YF = 0;
	double lidar_thetab = 0,lidar_thetar = 0;
	double xl,xr,yl,yr,zl,zr,hl,hr;
	if(flag == 0){//蓝场
			 xl = 0;//0
			 xr = 3;//2
    		 yl = -2.5;//-3
			 yr = 0;//3
			 zl = 0;//0
			 zr = 2;//2
			 hl = 0.3;
			 hr = 2;	

	}
	else{//红场
			 xl = 0;//0
			 xr = 3;//2
    		 yl = 0;//-3
			 yr = 2.5;//3
			 zl = 0;//0
			 zr = 2;//2
			 hl = 0.3;
			 hr = 2;	

	}

//------------XF 1827.67 YF 1717.08
	cylinder_list* cylinders = new cylinder_list();
	f1<<"**********";
	for (i = 0; i < 5;) {
		f1<< "i=" << i << endl;
		f1<< "寻找 x " << xl << " " << xr << " y " << yl << " " << yr << endl;
		f1<<"direction"<<direction<<endl;
		if (direction == 2) {//换方向超过两次
			f1<< "翻转了两次结束" << endl;
			break;
		}
		if (i == 0) {//找第一个,y不要切割
			j = segment_cylinder(cloud, xl, xr, yl, yr, zl, zr, hl, hr,cylinders, i, X, Y, Height, XF, YF, cloud_cylinders, direction, 0);
		}
		else {//y需要切割,找下一个
			j = segment_cylinder(cloud, xl, xr, yl, yr, zl, zr, hl, hr,cylinders, i, X, Y, Height, XF, YF, cloud_cylinders, direction, 1);
		}
		if (j == 2) {

			f1<< "找到了"  <<" X "<< X << " Y " << Y <<" Height "<<Height<<endl;
	
			if (direction) {
				xr = X / 1000;
				xl = X / 1000 - 2;
				yl = Y / 1000 - 2.6;
				yr = Y / 1000 + 2.6;
			}
			else {
				xl = X / 1000;
				xr = X / 1000 + 2;
				yl = Y / 1000 - 2.6;
				yr = Y / 1000 + 2.6;
			}
			// if(find_cldrs == 1){
			// 	break;
			// }
			// else{
				continue;
			// }

		}
		if (j == 0) {//切割点云不是圆柱或者切割出来的‘圆柱’周围有干扰就换方向
			//f1<< "没找到 " << xl << " " << xr << " y " << yl << " " << yr << endl;

			if (direction) {//注意j==0只多找了一次，避免了空旷场地连续多个空白点云的情况
				//找下两个，y不需要切割
				if (segment_cylinder(cloud, xl - 2, xr - 2, yl, yr, zl, zr, hl, hr,cylinders, i, X, Y, Height, XF, YF, cloud_cylinders, direction,  0) == 2) {

					xr = X / 1000;
					xl = X / 1000 - 2;//１.4是旋转角度为0时相邻两柱子X方向的差值，旋转角为0时Ｘ的值是相邻柱子间最大的Ｘ间距
					yl = Y / 1000 - 2.6;//2.6是旋转角为90时相邻两柱子Ｙ方向的差值,旋转角为90时Ｙ的值是相邻柱子间最大的Ｙ间距
					yr = Y / 1000 + 2.6;


					f1<< "越过遮挡找到了"  <<" X "<< X << " Y " << Y <<" Height "<<Height<<endl;
					// if(find_cldrs == 1){
					// 	break;
					// }
					// else{
						continue;
					// }


				}


			}
			else {

				//找下两个，y不需要切割
				if (segment_cylinder(cloud, xl + 2, xr + 2, yl, yr, zl, zr, hl, hr,cylinders, i, X, Y, Height, XF, YF, cloud_cylinders, direction, 0) == 2) {
					xl = X / 1000;
					xr = X / 1000 + 2;
					yl = Y / 1000 - 2.6;
					yr = Y / 1000 + 2.6;
					f1<< "越过遮挡找到了" <<" X "<< X << " Y " << Y <<" Height "<<Height<<endl;
					// if(find_cldrs == 1){
					// 	break;
					// }
					// else{
						continue;
					// }

				}
			}


			f1<< "翻转一次" << endl;
			if (XF == 0) {//超出第五个柱子的情况，XF会在第一个柱子找到后初始化
				xr = 0;
				xl = xr - 2;
			}
			else {

				xr = XF / 1000;
				xl = xr - 2;
				yl = YF / 1000 - 2.6;
				yr = YF / 1000 + 2.6;

			}

			direction++;
			continue;
		}


	}

	double  post_thetab,post_thetar;
	f1<<"index "<<cylinders->nodeIndex<<endl;
	if (cylinders->nodeIndex >= 4) {

		cylinders->get_coordinate(lider_Xb, lider_Yb, lidar_thetab, rob_Xb, rob_Yb,lider_Xr, lider_Yr, lidar_thetar, rob_Xr, rob_Yr, offset_X, offset_Y);
		if ((abs(lider_Xb) < 10000 && abs(lider_Yb) < 10000)||(abs(lider_Xr) < 10000 && abs(lider_Yr) < 10000)) {
			get_Post(cloud, lider_Xb, lider_Yb, lidar_thetab,lider_Xr, lider_Yr, lidar_thetar, cloud_cylinders,  post_thetab,post_thetar);
		}
		// if (lider_X < 10000 && lider_Y < 10000) {
		// 	f1 << lider_X << " " << lider_Y << endl;
		// }
		//f1 << "rob_X " << (int)(-rob_Y) << " rob_Y " << (int)rob_X <<" post_theta "<<(int)(post_theta*10)<< endl;
		//f1 << "rob_X " << (int)(-rob_Y) << " rob_Y " << (int)rob_X << " lider_X " << (int)lider_Y << " lider_Y " << (int)lider_X;
	}
	// else if(cylinders->nodeIndex == 1){
	// 	//f1<<"X"<<lider_X<<"Y"<<lider_Y<<endl;
	// 	rob_X = X;
	// 	rob_Y = Y;
	// }
	int ROB_Yb;
	int ROB_Xb;
	int LIDAR_Yb;
	int LIDAR_Xb;
	int POST_THETAb;
	int LIDAR_THETAb;

	int ROB_Yr;
	int ROB_Xr;
	int LIDAR_Yr;
	int LIDAR_Xr;
	int POST_THETAr;
	int LIDAR_THETAr;
		//kalmanflag = 1;
	
	if(XF != 0 && YF != 0){
		cout<<"------------XF "<<XF<<" YF "<<-YF<<"Height"<<Height<<endl;
		send_buf[0] = 0xaa;
		send_buf[1] = (int)XF >> 8;
		send_buf[2] = (int)XF;
		send_buf[3] = (int)YF >> 8;
		send_buf[4] = (int)YF;
		send_buf[5] = 0;
		send_buf[6] = 0;
		send_buf[7] = 0;
		send_buf[8] = 0;
		send_buf[9] = 0;
		send_buf[10] = 0;
		send_buf[11] = 0;
		send_buf[12] = 0;
		send_buf[13] = 0xcc;
		//send_data();
	}
	if ((abs(rob_Xb) < 10000 && abs(rob_Yb) < 10000 )||(abs(rob_Xr) < 10000 && abs(rob_Yr) < 10000 )) {
		// KalmanDo(KalmanRobX, rob_X, kalmanflag); //识别成功卡尔曼滤波
		// KalmanDo(KalmanRobY, rob_Y, kalmanflag);
		// KalmanDo(KalmanRobTheta, post_theta, kalmanflag);
		// KalmanDo(KalmanLidarX, lider_X, kalmanflag);
		// KalmanDo(KalmanLidarY, lider_Y, kalmanflag);
		// KalmanDo(KalmanLidarTheta, lidar_theta, kalmanflag);
		// kalmanflag = 1;
		// ROB_Y = (int)(-YF);
		// ROB_X = (int)XF;
		ROB_Yb = (int)(-rob_Yb);
		ROB_Xb = (int)rob_Xb;
		LIDAR_Yb = (int)lider_Yb;
		LIDAR_Xb = (int)lider_Xb;
		POST_THETAb = (int)(post_thetab * 10);
		LIDAR_THETAb = (int)(lidar_thetab * 10);

		ROB_Yr = (int)(-rob_Yr);
		ROB_Xr = (int)rob_Xr;
		LIDAR_Yr = (int)lider_Yr;
		LIDAR_Xr = (int)lider_Xr;
		POST_THETAr = (int)(post_thetar * 10);
		LIDAR_THETAr = (int)(lidar_thetar * 10);
		f1<<" rob_Xb "<<ROB_Xb<<" rob_Yb "<<ROB_Yb<<" post_thetab "<<POST_THETAb<<" lider_Xb "<<LIDAR_Xb<<" lider_Yb "<<LIDAR_Yb<<" lidar_thetab "<<LIDAR_THETAb<<endl;
		//cout<<" ############rob_Xb "<<ROB_Xb<<" rob_Yb "<<ROB_Yb<<" post_thetab "<<POST_THETAb<<" lider_Xb "<<LIDAR_Xb<<" lider_Yb "<<LIDAR_Yb<<" lidar_thetab "<<LIDAR_THETAb<<endl;

		f1<<" ------------rob_Xr "<<ROB_Xr<<" rob_Yr "<<ROB_Yr<<" post_thetar "<<POST_THETAr<<" lider_Xr "<<LIDAR_Xr<<" lider_Yr "<<LIDAR_Yr<<" lidar_thetar "<<LIDAR_THETAr<<endl;
		cout<<" ############rob_Xr "<<ROB_Xr<<" rob_Yr "<<ROB_Yr<<" post_thetar "<<POST_THETAr<<" lider_Xr "<<LIDAR_Xr<<" lider_Yr "<<LIDAR_Yr<<" lidar_thetar "<<LIDAR_THETAr<<endl;
		send_buf[0] = 0xaa;
		send_buf[1] = POST_THETAb >> 8;
		send_buf[2] = POST_THETAb;
		send_buf[3] = POST_THETAr >> 8;
		send_buf[4] = POST_THETAr;
		send_buf[5] = ROB_Xb >> 8;
		send_buf[6] = ROB_Xb;
		send_buf[7] = ROB_Yb >> 8;
		send_buf[8] = ROB_Yb;
		send_buf[9] = ROB_Xr >> 8;
		send_buf[10] = ROB_Xr;
		send_buf[11] = ROB_Yr >> 8;
		send_buf[12] = ROB_Yr;
		send_buf[13] = 0xcc;
		//send_data();
		//f1<<" p "<<KalmanRobX.p<<" k "<<KalmanRobX.k<<" flag "<<kalmanflag<<endl;
		//f1 << " lider_X1 " << LIDAR_X << " lider_Y1 " << LIDAR_Y << " LIDAR_THETA " << LIDAR_THETA << " rob_X " << ROB_X << " rob_Y " << ROB_Y << " post_theta " << POST_THETA << endl;
	}
	// else{//否则kalmanflag置0
	// 	kalmanflag=0;

	// }


	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("3D Viewer0"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer1"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer2"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("3D Viewer3"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4(new pcl::visualization::PCLVisualizer("3D Viewer4"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer5(new pcl::visualization::PCLVisualizer("3D Viewer5"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer6(new pcl::visualization::PCLVisualizer("3D Viewer6"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer7(new pcl::visualization::PCLVisualizer("3D Viewer7"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer8(new pcl::visualization::PCLVisualizer("3D Viewer8"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer9(new pcl::visualization::PCLVisualizer("3D Viewer9"));
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer10(new pcl::visualization::PCLVisualizer("3D Viewer10"));

	// viewer0->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud0");
	// viewer1->addPointCloud<pcl::PointXYZ>(partition(cloud, -10,10, -5, 5, zl, zr), "sample cloud1");
	// viewer2->addPointCloud<pcl::PointXYZ>(cloud_cylinders[0], "sample cloud2");
	// viewer3->addPointCloud<pcl::PointXYZ>(cloud_cylinders[1], "sample cloud3");
	// viewer4->addPointCloud<pcl::PointXYZ>(cloud_cylinders[2], "sample cloud4");
	// viewer5->addPointCloud<pcl::PointXYZ>(cloud_cylinders[3], "sample cloud5");
	// viewer6->addPointCloud<pcl::PointXYZ>(cloud_cylinders[4], "sample cloud6");

	// viewer7->addPointCloud<pcl::PointXYZ>(cloud_cylinders[5], "sample cloud7");
	// viewer8->addPointCloud<pcl::PointXYZ>(cloud_cylinders[6], "sample cloud8");
	// viewer7->addPointCloud<pcl::PointXYZ>(cloud_cylinders[7], "sample cloud9");
	// viewer8->addPointCloud<pcl::PointXYZ>(cloud_cylinders[8], "sample cloud10");

	// viewer0->setBackgroundColor(0, 0, 0);
	// viewer1->setBackgroundColor(0, 0, 0);
	// viewer2->setBackgroundColor(0, 0, 0);
	// viewer3->setBackgroundColor(0, 0, 0);
	// viewer4->setBackgroundColor(0, 0, 0);
	// viewer5->setBackgroundColor(0, 0, 0);
	// viewer6->setBackgroundColor(0, 0, 0);
	// viewer7->setBackgroundColor(0, 0, 0);
	// viewer8->setBackgroundColor(0, 0, 0);
	// viewer9->setBackgroundColor(0, 0, 0);
	// viewer10->setBackgroundColor(0, 0, 0);


	// viewer0->addCoordinateSystem(1.0);
	// viewer1->addCoordinateSystem(1.0);
	// viewer2->addCoordinateSystem(1.0);
	// viewer3->addCoordinateSystem(1.0);
	// viewer4->addCoordinateSystem(1.0);
	// viewer5->addCoordinateSystem(1.0);
	// viewer6->addCoordinateSystem(1.0);
	// viewer7->addCoordinateSystem(1.0);
	// viewer8->addCoordinateSystem(1.0);
	// viewer9->addCoordinateSystem(1.0);
	// viewer10->addCoordinateSystem(1.0);

	// viewer0->initCameraParameters();
	// viewer1->initCameraParameters();
	// viewer2->initCameraParameters();
	// viewer3->initCameraParameters();
	// viewer4->initCameraParameters();
	// viewer5->initCameraParameters();
	// viewer6->initCameraParameters();
	// viewer7->initCameraParameters();
	// viewer8->initCameraParameters();
	// viewer9->initCameraParameters();
	// viewer10->initCameraParameters();

	// while (!viewer1->wasStopped())
	// {
	// 	viewer0->spinOnce(100);
	// 	viewer1->spinOnce(100);
	// 	viewer2->spinOnce(100);
	// 	viewer3->spinOnce(100);
	// 	viewer4->spinOnce(100);
	// 	viewer5->spinOnce(100);
	// 	viewer6->spinOnce(100);
	// 	viewer7->spinOnce(100);
	// 	viewer8->spinOnce(100);
	// 	viewer9->spinOnce(100);
	// 	viewer10->spinOnce(100);
	// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }





}

int main(int argc, char** argv)
{

	// printf("开始程序");
	// f1 << endl;
	//    fd = UART0_Open(fd,ss);   
	//      do
	// 	{  
	// 		err = UART0_Init(fd,115200,0,8,1,'N');  
	// 		printf("Set Port Exactly!\n");  
	// 	}while(FALSE == err || FALSE == fd);  

		//卡尔曼初始化
		// kalmanflag=0;
		// KalmanInit();
	int flag=-1;
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	recive_data(flag);
	if(flag==0||flag==1){
	ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, boost::bind(&velodyneViewerCallback,_1,flag));

	ros::spin();
	}

	return (0);


}





