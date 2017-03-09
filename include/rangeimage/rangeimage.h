#ifndef RANGEIMAGE_H
#define RANGEIMAGE_H

#include <iostream>
#include <fstream>
#include <sstream>
#include "../../src/ElasticFusion.h"

#include <time.h>
//#include "../../src/LineParamEstimator.h"
//#include "time.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"

#include <vector>
#include <map>
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <vector>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <rangeimage/MainController.h>
#include "../Utils/Parse.h"
#include <vector>
#include "../../src/Tools/GUI.h"
#include "../../src/Tools/GroundTruthOdometry.h"
#include "../../src/Tools/RawLogReader.h"
#include "../../src/Tools/LiveLogReader.h"


using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBL PointType;
typedef pair<int ,int> PAIR;

static int count_global=0;
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    //pub_ok_ = n_.advertise<std_msgs::String>("bdbox_start", 1);
    pub_ = n_.advertise<std_msgs::Float64MultiArray>("bdbox_fixed", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("bdbox", 1000, &SubscribeAndPublish::chatterCallback, this);
    //sub_head_ = n_.subscribe("file_index", 1000, &SubscribeAndPublish::indexCallback, this);
    //sub_median_ = n_.subscribe("file_index_each", 1000, &SubscribeAndPublish::medianCallback, this);
//    &mainController=new MainController();
//    MainController mainController;
  }

  void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void indexCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  //void medianCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void initializeMainController();

  int dir_num;
private:
  MainController mainController;
  ros::NodeHandle n_;
  //ros::Publisher pub_ok_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  //ros::Subscriber sub_head_;
  //ros::Subscriber sub_median_;

  ifstream fidin;

  pcl::PointCloud<PointType>::Ptr pointcloud_global;

  pcl::PointCloud<PointType>::Ptr object1;
  pcl::PointCloud<PointType>::Ptr object2;
  pcl::PointCloud<PointType>::Ptr object3;
  pcl::PointCloud<PointType>::Ptr object4;
  pcl::PointCloud<PointType>::Ptr object5;
  pcl::PointCloud<PointType>::Ptr object6;
  pcl::PointCloud<PointType>::Ptr object7;
  pcl::PointCloud<PointType>::Ptr object8;
  pcl::PointCloud<PointType>::Ptr object9;
  pcl::PointCloud<PointType>::Ptr object10;
  pcl::PointCloud<PointType>::Ptr object11;
  pcl::PointCloud<PointType>::Ptr object12;
  //pcl::PointCloud<PointType>::Ptr object13;
  
  map<int, vector<int> > map1;
  map<int, vector<int> > map2;
  map<int, vector<int> > map3;
  map<int, vector<int> > map4;
  map<int, vector<int> > map5;
  map<int, vector<int> > map6;
  map<int, vector<int> > map7;
  map<int, vector<int> > map8;
  map<int, vector<int> > map9;
  map<int, vector<int> > map10;
  map<int, vector<int> > map11;
  map<int, vector<int> > map12;
  //map<int, vector<int>> map13;

};//End of class SubscribeAndPublish

extern float f;
extern float cx,cy;

extern int cnt;
extern int count_mine;
extern int count_zong;
extern int cnt_pcd;
extern int threshold_merge;
extern bool flag_break;

bool agree1(unsigned char &label, vector<float> &parameters, Point3f &data);
void equation3(float *A, float *B, float *C, float *D, float x1, float y1, float z1, float x2, float y2, float z2,  float x3, float y3, float z3);
void myRansac(unsigned char &label, int &cntSave1, vector<Point3f> &data, int &left, int &right, int &top, int &bottom, vector<float> &parameters1, cv::Mat depth_image, vector<float> &parameters_get1);
pcl::PointCloud<PointType> dem_plain(cv::Mat depth_image, unsigned char label, int &left, int &right, int &top, int &bottom, double &bond_left, double &bond_right, double &bond_top, double &bond_bottom);
bool cmp_by_value(const PAIR& lhs, const PAIR& rhs);
string int2str(double n);
pcl::PointCloud<PointType> depth2cloud( cv::Mat depth_image, /*cv::Mat rgb_image*/unsigned char label, int left, int right, int top, int bottom, double& bond_left, double& bond_right, double& bond_top, double& bond_bottom);
cv::Scalar GetRandomColor(int label);
void cvText(IplImage* img, const char* text, int x, int y);
void LabelColor(const cv::Mat& _labelImg, IplImage* image,int label);
void Seed_Filling(cv::Mat& binImg, int span);

void equation3(float *A, float *B, float *C, float *D, float x1, float y1, float z1, float x2, float y2, float z2,  float x3, float y3, float z3);
void estimate(std::vector<Point3f *> &data,std::vector<float> &parameters);
bool agree(std::vector<float> &parameters, Point3f &data);

#endif
