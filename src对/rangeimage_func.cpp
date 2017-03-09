#include "../include/rangeimage/rangeimage.h"

float f = 570.3;
float cx = 320.0, cy = 240.0;

//int cnt=407;
//int count_mine=739;
int cnt=0;
int count_mine=0;
int count_zong=0;
int cnt_pcd=0;
int threshold_merge=34;
bool flag_break=true;

string int2str(double n)
{
    stringstream ss;
    string s;
    ss << n;
    ss >> s;

    return s;
}

//the higher the fronter
bool cmp_by_value(const PAIR& lhs, const PAIR& rhs) {
 return lhs.second > rhs.second;
}

pcl::PointCloud<PointType> depth2cloud( cv::Mat depth_image, /*cv::Mat rgb_image*/unsigned char label, int left, int right, int top, int bottom, double& bond_left, double& bond_right, double& bond_top, double& bond_bottom) {

    pcl::PointCloud<PointType> pointCloud;
    int aaa=0;

        for ( int x = left; x < right; ++ x ) {
            for ( int y = top; y < bottom; ++ y ) {
                PointType pt;
                if ( depth_image.at<unsigned short>(y, x) == 0 ) {
                    pt.x = std::numeric_limits<float>::quiet_NaN();
                    pt.y = std::numeric_limits<float>::quiet_NaN();
                    pt.z = std::numeric_limits<float>::quiet_NaN();
                }
                else
                {
                    pt.z = depth_image.at<unsigned short>(y, x)/1000.0;
                    pt.y = (y-cy)*pt.z/f;
                    pt.x = (x-cx)*pt.z/f;

                    if (label==2)
                    {
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;
                    }
                    else if (label==5)
                    {
                        pt.r = 255;
                        pt.g = 255;
                        pt.b = 0;
                    }
                    else if (label==9)
                    {
                        pt.r = 0;
                        pt.g = 255;
                        pt.b = 0;
                    }
                    else if (label==12)
                    {
                        pt.r = 0;
                        pt.g = 255;
                        pt.b = 255;
                    }
                    else if (label==15)
                    {
                        pt.r = 0;
                        pt.g = 0;
                        pt.b = 255;
                    }
                    else if (label==16)
                    {
                        pt.r = 160;
                        pt.g = 32;
                        pt.b = 240;
                    }
                    else
                    {
                        pt.r = 0;
                        pt.g = 0;
                        pt.b = 0;
                    }
//                    pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
//                    pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
//                    pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
                    pt.label = label;
                }
                if (aaa==0)
                {
                    bond_left=pt.x;
                    bond_right=pt.x;
                    bond_top=pt.y;
                    bond_bottom=pt.y;
                    aaa++;
                }
                else
                {
                    if (pt.x<bond_left)
                    {
                        bond_left=pt.x;
                    }
                    if (pt.x>bond_right)
                    {
                        bond_right=pt.x;
                    }
                    if (pt.y<bond_top)
                    {
                        bond_top=pt.y;
                    }
                    if (pt.y>bond_bottom)
                    {
                        bond_bottom=pt.y;
                    }

                }

                pointCloud.points.push_back( pt );
            }
        }

//        pointCloud.width  = count;
//        pointCloud.height = 1;
//        pointCloud.is_dense = true;//false;
//                pointCloud.width  = depth_image.cols;
//                pointCloud.height = depth_image.rows;
//                pointCloud.is_dense = true;//false;
        pointCloud.width  = right-left;
        pointCloud.height = bottom-top;
        pointCloud.is_dense = true;//false;

    return pointCloud;
}

cv::Scalar GetRandomColor(int label)
{
//   uchar r = 255 * (rand()/(1.0 + RAND_MAX));
//   uchar g = 255 * (rand()/(1.0 + RAND_MAX));
//   uchar b = 255 * (rand()/(1.0 + RAND_MAX));

    uchar r,g,b;
    if (label==2)
    {
        r = 255;
        g = 0;
        b = 0;
    }
    else if (label==5)
    {
        r = 255;
        g = 255;
        b = 0;
    }
    else if (label==9)
    {
        r = 0;
        g = 255;
        b = 0;
    }
    else if (label==12)
    {
        r = 0;
        g = 255;
        b = 255;
    }
    else if (label==15)
    {
        r = 0;
        g = 0;
        b = 255;
    }
    else if (label==16)
    {
        r = 160;
        g = 32;
        b = 240;
    }
    else
    {
        r = 0;
        g = 0;
        b = 0;
    }
   return cv::Scalar(b,g,r) ;
}

void cvText(IplImage* img, const char* text, int x, int y)
{
    CvFont font;

    double hscale = 1.0;
    double vscale = 1.0;
    int linewidth = 2;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);

    CvScalar textColor =cvScalar(0,255,0);
    CvPoint textPos =cvPoint(x, y);

    cvPutText(img, text, textPos, &font,textColor);
}

void LabelColor(const cv::Mat& _labelImg, IplImage* image,int label)
{
   if (_labelImg.empty() ||
       _labelImg.type() != CV_32SC1)
   {
       cout<<"return......................"<<endl;
       return ;
   }

   std::map<int, CvScalar> colors ;

   int rows = _labelImg.rows ;
   int cols = _labelImg.cols ;

   CvPoint centerpoint;
   for (int i = 0; i < rows; i++)
   {
       const int* data_src = (int*)_labelImg.ptr<int>(i) ;
       for (int j = 0; j < cols; j++)
       {
           int pixelValue = data_src[j] ;

           if (pixelValue >= 1)
           {
               if (colors.count(pixelValue) <= 0)
               {
                   colors[pixelValue] = GetRandomColor(label) ;
               }

               cv::Scalar color = colors[pixelValue] ;
               centerpoint.x=j;
               centerpoint.y=i;
               cvCircle( image, centerpoint ,1 , color,1, 8, 3 );
           }
       }
   }
}
void Seed_Filling(cv::Mat& binImg, int span)
{
  int rows = binImg.rows - 1 ;
  int cols = binImg.cols - 1 ;
  int label=1;
  for (int i = 1; i < rows-1; i++)
  {
      int* data= binImg.ptr<int>(i) ;
      for (int j = 1; j < cols-1; j++)
      {
          if (data[j] == 1)
          {
              std::stack<std::pair<int,int> > neighborPixels ;
              neighborPixels.push(std::pair<int,int>(i,j)) ;     // pixel position: <i,j>
//              cout<<"neighborPixels.size()..............."<<neighborPixels.size()<<endl;
              ++label ;  // begin with a new label
              while (!neighborPixels.empty())
              {
                  // get the top pixel on the stack and label it with the same label
                  std::pair<int,int> curPixel = neighborPixels.top() ;
                  int curX = curPixel.first ;
                  int curY = curPixel.second ;
                  binImg.at<int>(curX, curY) = label ;

                  // pop the top pixel
                  neighborPixels.pop() ;

                  for (int m=curX-span;m<curX+span+1;m++)
                  {
                      if ((m<0)||(m>479))
                      {
                          continue;
                      }
                      for (int n=curY-span;n<curY+span+1;n++)
                      {
                          if ((n<0)||(n>639))
                          {
                              continue;
                          }
                          if ((m==curX)&&(n==curY))
                          {
                              continue;
                          }
                          if (binImg.at<int>(m,n)==1)
                          {
                              neighborPixels.push(pair<int,int>(m,n));
                          }
                      }
                  }
              }
          }
      }
  }
}

//void Two_Pass(cv::Mat& binImg,int span)    //Áœ±éÉšÃè·š
//{
//  if (binImg.empty())
//  {
//    return;
//  }

//  // µÚÒ»žöÍšÂ·

//  span=30;
//  int label = 1;
//  std::vector<int> labelSet;
//  labelSet.push_back(0);
//  labelSet.push_back(1);

//  int rows = binImg.rows - 1;
//  int cols = binImg.cols - 1;
//  for (int i = 1; i < rows; i++)
//  {
////    int* data_preRow = binImg.ptr<int>(i-1);
//    int* data_curRow = binImg.ptr<int>(i);
//    for (int j = 1; j < cols; j++)
//    {
//      if (data_curRow[j] == 1)
//      {
//        std::vector<int> neighborLabels;
////        neighborLabels.reserve(2);

//        for (int m=i-span;m<i;m++)
//        {
//            for (int n=j-span;n<j;n++)
//            {
//                int neighborPixel = binImg.at<int>(m,n);
////                int upPixel = data_preRow[j];
//                if ( neighborPixel >= 1)
//                {
//                    neighborLabels.push_back(neighborPixel);
//                }
////                if (upPixel > 1)
////                {
////                    neighborLabels.push_back(upPixel);
////                }
//            }
//        }

//        if (neighborLabels.empty())
//        {
//          labelSet.push_back(++label);  // ²»Á¬Íš£¬±êÇ©+1
//          data_curRow[j] = label;
//          labelSet[label] = label;
//        }
//        else
//        {
//          std::sort(neighborLabels.begin(), neighborLabels.end());
//          int smallestLabel = neighborLabels[0];
//          data_curRow[j] = smallestLabel;

//          // ±£Žæ×îÐ¡µÈŒÛ±í
//          for (size_t k = 1; k < neighborLabels.size(); k++)
//          {
//            int tempLabel = neighborLabels[k];
//            int& oldSmallestLabel = labelSet[tempLabel];
//            if (oldSmallestLabel > smallestLabel)
//            {
//              labelSet[oldSmallestLabel] = smallestLabel;
//              oldSmallestLabel = smallestLabel;
//            }
//            else if (oldSmallestLabel < smallestLabel)
//            {
//              labelSet[smallestLabel] = oldSmallestLabel;
//            }
//          }
//        }
//      }
//    }
//  }

//  cout<<"in3................"<<endl;

//  // žüÐÂµÈŒÛ¶ÔÁÐ±í
//  // œ«×îÐ¡±êºÅžøÖØžŽÇøÓò
//  for (size_t i = 2; i < labelSet.size(); i++)
//  {
//    int curLabel = labelSet[i];
//    int preLabel = labelSet[curLabel];
//    while (preLabel != curLabel)
//    {
//      curLabel = preLabel;
//      preLabel = labelSet[preLabel];
//    }
//    labelSet[i] = curLabel;
//  }  ;

//  for (int i = 0; i < rows; i++)
//  {
//    int* data = binImg.ptr<int>(i);
//    for (int j = 0; j < cols; j++)
//    {
//      int& pixelLabel = data[j];
//      pixelLabel = labelSet[pixelLabel];
//    }
//  }
//}
