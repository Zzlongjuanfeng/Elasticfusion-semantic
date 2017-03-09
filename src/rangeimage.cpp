#include "../include/rangeimage/rangeimage.h"
extern float f;
extern float cx,cy;

extern int cnt;
extern int count_mine;
extern int count_zong;
extern int cnt_pcd;
extern int threshold_merge;

// setup image directories
//string dir_root="/media/disk1/xr/Now/data_klg/";
string dir_root="/home/zxf/Dataset/sherry/";

//bool cmp_by_value(const PAIR& lhs, const PAIR& rhs);
//string int2str(double n);
//pcl::PointCloud<PointType> depth2cloud( cv::Mat depth_image, int left, int right, int top, int bottom);
//cv::Scalar GetRandomColor();
//void LabelColor(const cv::Mat& _labelImg, IplImage* image);
//void Seed_Filling(cv::Mat& binImg, int span);
void SubscribeAndPublish::initializeMainController()
{
    cout << "begin initial......" << endl;
    dir_num=4;
    string fidin_dir=dir_root+int2str(dir_num);
    fidin_dir += "/";
    fidin_dir += int2str(dir_num);
    fidin_dir += ".txt";
    fidin.open(fidin_dir.c_str());

    mainController.initializeeFusion();
    cout << "end initial......" << endl;
}

void SubscribeAndPublish::chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    clock_t start_time_chatter = clock();
    // ------------------------------------------------------------------
    // -------------------Process input ROS information------------------
    // ------------------------------------------------------------------
  ROS_INFO("I heard in chatterCallback: [%f %f %f]", msg->data[0], msg->data[1], msg->data[2]);

//  if (msg->data[0]==-1)
//  {
//        int index_file_depth=msg->data[1];
//        int index_file_rgb=msg->data[2];
//        char tmp1[1000];
//        double a,b,c,d,e,f,g,h,i,k;
//        int j,l;
//        fidin.getline(tmp1,1000);
//        sscanf(tmp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %d.png %lf %d.jpg", &a, &b, &c, &d, &e, &f, &g, &h, &i, &j, &k, &l);

//        while(index_file_depth<j)
//        {

//      int index_file_depth=msg->data[1];
//      int index_file_rgb=msg->data[2];
      int index_file_depth=0;
      int index_file_rgb=0;
      char tmp1[1000];
      double a,b,c,d,e,f,g,h,i,k;
      int j,l;
      while(index_file_depth<2000)
      {
//          while(index_file_depth<900)
//          {
//          fidin.getline(tmp1,1000);
//          index_file_depth++;
//          }
          fidin.getline(tmp1,1000);
          sscanf(tmp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %d_depth.png %lf %d.jpg", &a, &b, &c, &d, &e, &f, &g, &h, &i, &j, &k, &l);

          index_file_depth=j;
          index_file_rgb=l;

      string filename_c="/rgb/";
      string filename_d="/depth/";
      string filename_b="/res/binImg/";
      string file_cnt_depth;

      string file_cnt_rgb;
      string filename_tail_rgb=".jpg";
      string filename_tail_depth=".png";
      string FILENAME_depth="";
      string FILENAME_color="";
      string FILENAME_binImg="";

      file_cnt_depth=int2str(index_file_depth);
      file_cnt_rgb=int2str(index_file_rgb);

      FILENAME_depth="";
      FILENAME_depth+=dir_root;
      FILENAME_depth+=int2str(dir_num);
      FILENAME_depth+=filename_d;
      FILENAME_depth=FILENAME_depth+file_cnt_depth;
      FILENAME_depth+=filename_tail_depth;

      FILENAME_color="";
      FILENAME_color+=dir_root;
      FILENAME_color+=int2str(dir_num);
      FILENAME_color+=filename_c;
      FILENAME_color=FILENAME_color+file_cnt_rgb;
      FILENAME_color+=filename_tail_rgb;

      FILENAME_binImg="";
      FILENAME_binImg+=dir_root;
      FILENAME_binImg+=int2str(dir_num);
      FILENAME_binImg+=filename_b;
      FILENAME_binImg=FILENAME_binImg+file_cnt_rgb;
      FILENAME_binImg+=filename_tail_rgb;

      cv::Mat _binImg=cv::Mat(480,640,/*CV_32SC1*/CV_8UC3,Scalar(0,0,0));

      fstream _filebinImg;
      _filebinImg.open(FILENAME_binImg.c_str());

      cv::Mat image_in=cv::imread(FILENAME_color.c_str());
      cv::Mat depth_in = cv::imread(FILENAME_depth.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file

//      cout << FILENAME_color << endl;
//      cout << FILENAME_depth << endl;
//      cv::imshow("rgb",image_in);
//      cv::waitKey(0);
//      cv::imshow("depth",depth_in);
//      cv::waitKey(0);


      if (_filebinImg)
      {
          _binImg=cv::imread(FILENAME_binImg.c_str());
          mainController.run(index_file_rgb, dir_num, image_in, depth_in, _binImg/*, tran*/, true, count_global);
      }
      else
      {
          mainController.run(index_file_rgb, dir_num, image_in, depth_in, _binImg/*, tran*/, false, count_global);

          cout<<FILENAME_color<<endl;
          cout<<FILENAME_depth<<endl;
          cout<<FILENAME_binImg<<endl;

      }

          count_global++;

//      mainController.run(image_in, depth_in, _binImg/*, tran*/, false, count_global);


      cout<<"Picture:   "<<index_file_depth<<endl;

//      index_file_depth++;
//      index_file_rgb++;
      }
//  }

//  else
//  {
//  unsigned char label;
//  double left;
//  double right;
//  double top;
//  double bottom;
//  bool flag_bdboxfixed;

//  int dir_rec=msg->data[0];
//  if (dir_rec!=dir_num)
//  {
//      cout<<dir_num<<endl;
//      cout<<dir_rec<<endl;
//      dir_num=dir_rec;

//      string fidin_dir=dir_root+int2str(dir_num);
//      fidin_dir+="/";
//      fidin_dir+=int2str(dir_num);
//      fidin_dir+=".txt";
//      fidin.open(fidin_dir.c_str());
//  }
//  int index_file_depth=msg->data[1];
//  int index_file_rgb=msg->data[2];
//  cout<<"Picture:   "<<msg->data[3]<<endl;
//  double tempTx=msg->data[4];
//  double tempTy=msg->data[5];
//  double tempTz=msg->data[6];
//  double tempX=msg->data[7];
//  double tempY=msg->data[8];
//  double tempZ=msg->data[9];
//  double tempW=msg->data[10];

//  string filename_c="/rgb/";
//  string filename_d="/depth/";
//  string filename_p="/pcd/";
//  string filename_b="/res/binImg/";

//  string file_cnt_depth;
//  string file_cnt_rgb;
//  string filename_tail_rgb=".jpg";
//  string filename_tail_depth=".png";
//  string filename_tail_pcd=".pcd";

//  string FILENAME_depth="";
//  string FILENAME_color="";
//  string FILENAME_binImg="";

//  file_cnt_depth=int2str(index_file_depth);
//  file_cnt_rgb=int2str(index_file_rgb);

//  FILENAME_depth="";
//  FILENAME_depth+=dir_root;
//  FILENAME_depth+=int2str(dir_num);
//  FILENAME_depth+=filename_d;
//  FILENAME_depth=FILENAME_depth+file_cnt_depth;
//  FILENAME_depth+=filename_tail_depth;

//  FILENAME_color="";
//  FILENAME_color+=dir_root;
//  FILENAME_color+=int2str(dir_num);
//  FILENAME_color+=filename_c;
//  FILENAME_color=FILENAME_color+file_cnt_rgb;
//  FILENAME_color+=filename_tail_rgb;

//  FILENAME_binImg="";
//  FILENAME_binImg+=dir_root;
//  FILENAME_binImg+=int2str(dir_num);
//  FILENAME_binImg+=filename_b;
//  FILENAME_binImg=FILENAME_binImg+file_cnt_rgb;
//  FILENAME_binImg+=filename_tail_rgb;

////  cv::Mat depth_in(480,640,CV_16UC1,Scalar(0));
//  cv::Mat image_in=cv::imread(FILENAME_color.c_str());

//  //  cv::Mat image_in(image,0); //img拿到pImg的data
//  cv::Mat depth_in = cv::imread(FILENAME_depth.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file
//  //    depth_in.convertTo(depth_in, CV_32F,1.0/5000.0); // convert the image data to float type
//  cv::Mat _binImg=cv::Mat(480,640,/*CV_32SC1*/CV_8UC3,Scalar(0,0,0));

//  std_msgs::Float64MultiArray msg_pub;
//  vector<double> bdbox;
//  vector<vector<double>> bdboxes;
//  vector<vector<double>> bdboxes_return;
//  int bdbox_cnt=msg->data.size();

//  // ------------------------------------------------------------------
//  // -----Read pcd file or create example point cloud if not given-----
//  // ------------------------------------------------------------------
//  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
//  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
//  pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
//  pcl::PointCloud<PointType>::Ptr cloud_f (new pcl::PointCloud<PointType>);

//  for (int i=0;i<msg->data[bdbox_cnt-1];i++)
//  {
//      bdbox.clear();
//      for (int j=0;j<5;j++)
//      {
//          bdbox.push_back(msg->data[11+5*i+j-1]);
//      }
//      bdboxes.push_back(bdbox);
//  }
//  int pcd_cnt=1;
//  for (vector<vector<double>>::iterator it_bdbox=bdboxes.begin();it_bdbox!=bdboxes.end();it_bdbox++)
//  {
//      bdbox=*it_bdbox;
//      label=(unsigned char)bdbox[0];
//      left=bdbox[1];
//      right=bdbox[3];
//      top=bdbox[2];
//      bottom=bdbox[4];
//      flag_bdboxfixed=false;

////      IplImage* image=cvLoadImage(FILENAME_color.c_str());

//      string filename_save=dir_root;
//      filename_save+=int2str(dir_num);
//      filename_save+=filename_p;
//      filename_save+=file_cnt_depth;
//      filename_save+="_";
//      filename_save+=int2str(pcd_cnt);
//      filename_save+=filename_tail_pcd;
//      cout<<filename_save<<endl;

//      pcd_cnt++;
//      double bond_left=0.0;
//      double bond_right=0.0;
//      double bond_top=0.0;
//      double bond_bottom=0.0;

//      if ((label==16)&&((right-left<90)&&(bottom-top<120)))
//      {
//          bdbox.clear();
//          bdbox.push_back(label);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);

//          bdboxes_return.push_back(bdbox);
//          continue;
//      }
//      point_cloud = depth2cloud(depth_in,/*image_in*/label,left/*-20*/,right/*+20*/,top/*-20*/,bottom/*+20*/,bond_left, bond_right, bond_top, bond_bottom);

//      if (point_cloud.width==0)
//      {
//          bdbox.clear();
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);

//          bdboxes_return.push_back(bdbox);

//          continue;
//      }
//      *point_cloud_ptr = point_cloud;
////  cout<<"1: "<<point_cloud<<endl;

////      pcl::PCDWriter writer;
////                std::stringstream ss2;
////                ss2 << "/home/czhang/Faster_RCNN/Data_mine/1/pcd/depth2cloud_" << cnt << ".pcd";
////                writer.write<PointType> (ss2.str (), point_cloud, false);
//  //          pcl::IndicesPtr indices_z (new std::vector <int>);
//      pcl::PassThrough<PointType> passz;
//      passz.setInputCloud (point_cloud_ptr);
//      passz.setFilterFieldName ("z");
//      passz.setFilterLimits (0.0, 3.6);
//      //          pass.filter (*indices_z);
//      passz.filter (*cloud_filtered);

////  cout<<"2: "<<*cloud_filtered<<endl;

//  //          cout<<*cloud_filtered<<endl;
//  //          std::stringstream ss2;
//  //          ss2 << "/home/czhang/Faster_RCNN/Data_mine/1/pcd/cloud_zCut_" << cnt << ".pcd";
//  //          writer.write<PointType> (ss2.str (), *cloud_filtered, false); //*

//  ////          pcl::IndicesPtr indices_y (new std::vector <int>);
//  /// //          pcl::PassThrough<PointType> passy;
////          passy.setInputCloud (point_cloud_ptr);
////          passy.setFilterFieldName ("y");
////          passy.setFilterLimits (0.0, 1);
//////          pass.filter (*indices_y);
////          passy.filter (*cloud_filtered);
////          std::stringstream ss3;
////          ss3 << "/home/czhang/Faster_RCNN/Data_mine/1/pcd/cloud_yCut_" << cnt << ".pcd";
////          writer.write<PointType> (ss3.str (), *cloud_filtered, false); //*

//////          pcl::IndicesPtr indices_x (new std::vector <int>);
////          pcl::PassThrough<PointType> passx;
////          passx.setInputCloud (point_cloud_ptr);
////          passx.setFilterFieldName ("x");
////          passx.setFilterLimits (0.0, 1);
//////          pass.filter (*indices_x);
////          passx.filter (*cloud_filtered);
////          std::stringstream ss4;
////          ss4 << "/home/czhang/Faster_RCNN/Data_mine/1/pcd/cloud_xCut_" << cnt << ".pcd";
////          writer.write<PointType> (ss4.str (), *cloud_filtered, false); //*

////    // Create the filtering object: downsample the dataset using a leaf size of 1cm
////    pcl::VoxelGrid<PointType> vg;
////    vg.setInputCloud (cloud_filtered);
////    vg.setLeafSize (0.01f, 0.01f, 0.01f);
////    vg.filter (*cloud_filtered);
////    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
//////    std::stringstream ss1;
//////    ss1 << "/home/czhang/Faster_RCNN/Data_mine/1/pcd/cloud_filtered_" << cnt << ".pcd";
//////    writer.write<PointType> (ss1.str (), *cloud_filtered, false); //*

//      // ------------------------------------------------------------------
//      // -----------------Segment the object out of planes-----------------
//      // ------------------------------------------------------------------

////      // Create the segmentation object for the planar model and set all the parameters
////      pcl::SACSegmentation<PointType> seg;
////      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
////      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
////      pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
////      seg.setOptimizeCoefficients (true);
////      seg.setModelType (pcl::SACMODEL_PLANE);
////      seg.setMethodType (pcl::SAC_RANSAC);
////      seg.setMaxIterations (100);
////      seg.setDistanceThreshold (0.02);

////      int nr_points = (int) cloud_filtered->points.size ();
////      while ((cloud_filtered->points.size () > 0.7 * nr_points)&&(nr_points > 20))
////      {
////          // Segment the largest planar component from the remaining cloud
////          seg.setInputCloud (cloud_filtered);
////          seg.segment (*inliers, *coefficients);
////          if (inliers->indices.size () == 0)
////          {
////              std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
////              break;
////          }

////          // Extract the planar inliers from the input cloud
////          pcl::ExtractIndices<PointType> extract;
////          extract.setInputCloud (cloud_filtered);
////          extract.setIndices (inliers);
////          extract.setNegative (false);

////          // Get the points associated with the planar surface
////          extract.filter (*cloud_plane);
////          //      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

////          // Remove the planar inliers, extract the rest
////          extract.setNegative (true);
////          extract.filter (*cloud_f);
////          *cloud_filtered = *cloud_f;
////      }


//      // ------------------------------------------------------------------
//      // -----------------------Find biggest cluster-----------------------
//      // ------------------------------------------------------------------
//\
//      if (label!=21)
//      {
//      pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);

//      //cloud_filtered: object without planes
//      //  cout<<"3: "<<*cloud_filtered<<endl;
//      if (cloud_filtered->points.size () > 10/*(cloud_filtered->width!=0)&&(cloud_filtered->height!=0)*/)
//      {
//          // Creating the KdTree object for the search method of the extraction
//          pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
//          tree->setInputCloud (cloud_filtered);

//          std::vector<pcl::PointIndices> cluster_indices;
//          pcl::EuclideanClusterExtraction<PointType> ec;
//          ec.setClusterTolerance (0.15); // 2cm
//          ec.setMinClusterSize (20);
//          ec.setMaxClusterSize (130000);
//          ec.setSearchMethod (tree);
//          ec.setInputCloud (cloud_filtered);
//          ec.extract (cluster_indices);

//          cv::Point centerpoint;
////              Eigen::Vector3f point;
//          vector<float> point;

////                      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
////                      {

//          unsigned char* ptr;
//          if (cluster_indices.size()!=0)
//          {
//              std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();

//              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//              {
//                  point.push_back(cloud_filtered->points[*pit].x);
//                  point.push_back(cloud_filtered->points[*pit].y);
//                  point.push_back(cloud_filtered->points[*pit].z);
//                  cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
//                  point.clear();
//              }
//          }

//          cloud_cluster->width = cloud_cluster->points.size ();
//          cloud_cluster->height = 1;
//          cloud_cluster->is_dense = true;

//          std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

////    // Create the filtering object: downsample the dataset using a leaf size of 1cm
////    pcl::VoxelGrid<PointType> vg;
////    vg.setInputCloud (cloud_cluster);
////    vg.setLeafSize (0.01f, 0.01f, 0.01f);
////    vg.filter (*cloud_filtered);
////    std::cout << "PointCloud_Cluster after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

//      }

//      cv::Point centerpoint;
//      bond_left=640;
//      bond_right=0;
//      bond_top=480;
//      bond_bottom=0;
//      unsigned char* ptr;
//      for (int pit=0;pit!=cloud_cluster->size();pit++)
//      {
//          centerpoint.x=int(cloud_cluster->points[pit].x*f/cloud_cluster->points[pit].z+cx);
//          centerpoint.y=int(cloud_cluster->points[pit].y*f/cloud_cluster->points[pit].z+cy);
//          ptr=_binImg.data+centerpoint.y*_binImg.step + centerpoint.x * _binImg.elemSize();
//          ptr[0]=(unsigned char)(label*10);
//          if (centerpoint.x<bond_left)
//              bond_left=centerpoint.x;
//          if (centerpoint.x>bond_right)
//              bond_right=centerpoint.x;
//          if (centerpoint.y<bond_top)
//              bond_top=centerpoint.y;
//          if (centerpoint.y>bond_bottom)
//              bond_bottom=centerpoint.y;
//      }
//      cv::imwrite(FILENAME_binImg.c_str(),_binImg);

//      if (cloud_filtered->size()<10)
//      {
//          bdbox.clear();
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);

//          bdboxes_return.push_back(bdbox);

//          continue;
//      }
//      }

//      else
//      {
//      cv::Point centerpoint;
//      bond_left=640;
//      bond_right=0;
//      bond_top=480;
//      bond_bottom=0;
//      unsigned char* ptr;
//      for (int pit=0;pit!=cloud_filtered->size();pit++)
//      {
//          centerpoint.x=int(cloud_filtered->points[pit].x*f/cloud_filtered->points[pit].z+cx);
//          centerpoint.y=int(cloud_filtered->points[pit].y*f/cloud_filtered->points[pit].z+cy);
//          ptr=_binImg.data+centerpoint.y*_binImg.step + centerpoint.x * _binImg.elemSize();
//          ptr[0]=(unsigned char)(label*10);
//          if (centerpoint.x<bond_left)
//              bond_left=centerpoint.x;
//          if (centerpoint.x>bond_right)
//              bond_right=centerpoint.x;
//          if (centerpoint.y<bond_top)
//              bond_top=centerpoint.y;
//      if (centerpoint.y>bond_bottom)
//          bond_bottom=centerpoint.y;
//      }
//      cv::imwrite(FILENAME_binImg.c_str(),_binImg);

//      if (cloud_filtered->size()<10)
//      {
//          bdbox.clear();
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);
//          bdbox.push_back(0.0);

//          bdboxes_return.push_back(bdbox);

//          continue;
//      }
//      }

//      bdbox.clear();
//      bdbox.push_back(label);
//      bdbox.push_back(bond_left);
//      bdbox.push_back(bond_top);
//      bdbox.push_back(bond_right);
//      bdbox.push_back(bond_bottom);

//      bdboxes_return.push_back(bdbox);


//////      pcl::io::savePCDFileASCII (filename_save, *cloud_cluster);
//  }

//  cv::imwrite(FILENAME_binImg.c_str(),_binImg);

////  string str="/home/kaffe/catkin_ws/src/rangeimage/1.jpg";
////  cv::imshow("binImg",_binImg);
////  cv::waitKey(0);
//  //          pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, tran);

//  clock_t end_time_chatter=clock();
//  cout<< "Running time in chatterCallback 1 is: "<<static_cast<double>(end_time_chatter-start_time_chatter)/CLOCKS_PER_SEC*1000<<"ms"<<endl;//输出运行时间

//  start_time_chatter=clock();
//  Eigen::Matrix3f R_transform = Eigen::Quaternionf(tempW, tempX, tempY, tempZ).toRotationMatrix();
//  Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
//  Eigen::Vector3f T_transform=Eigen::Vector3f(tempTx, tempTy, tempTz);
//  trans.block(0,0,3,3) = R_transform;
//  trans.block(0,3,3,1) = T_transform;

//  Eigen::Matrix4f transform_base;
//  transform_base<<  0.0008,   -1.0000 ,   0.0000 ,  -0.0400,
//          0.0008,    0.0000,   -1.0000,    0.0000,
//          1.0000,    0.0008 ,   0.0008 ,   0.0000,
//          0 ,        0  ,       0 ,   1.0000;

//  Eigen::Matrix4f tran=trans;//*transform_base.inverse();

//  mainController.run(index_file_rgb, dir_num,image_in, depth_in, _binImg/*, tran*/, true, count_global);

//  end_time_chatter=clock();
//  count_global++;

//  cout<< "Running time in chatterCallback 2 is: "<<static_cast<double>(end_time_chatter-start_time_chatter)/CLOCKS_PER_SEC*1000<<"ms"<<endl;//输出运行时间

//  if (flag_bdboxfixed==false)
//  {
//      vector<double> data_fixed1;

//      for (int i=0;i<bdboxes_return.size();i++)
//      {
//          for (int j=0;j<5;j++)
//          {
//              data_fixed1.push_back(bdboxes_return[i][j]);
//          }
//      }
//      bdboxes_return.clear();

//      msg_pub.data = data_fixed1;
//  }

//  pub_.publish(msg_pub);

//  cnt++;
//  }
}

// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  ros::init(argc, argv, "subscriber_and_publish");
//  ros::NodeHandle n;
//  ros::Subscriber sub = n.subscribe("bdbox", 1000, chatterCallback);
//  ros::Subscriber sub_head = n.subscribe("file_index", 1000, indexCallback);

  cout<<"start............................"<<endl;
  SubscribeAndPublish SAPObject;
  SAPObject.initializeMainController();
//  ros::spin();
  while (ros::ok())
  {
    cout << "handle event......" <<endl;
    ros::spinOnce();                   // Handle ROS events
//    r.sleep();
  }

}

