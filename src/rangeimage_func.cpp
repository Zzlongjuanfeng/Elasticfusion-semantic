#include "../include/rangeimage/rangeimage.h"

#define angleThreshold 0.8
#define  MAX_ITERATOR 500//前方1*1拟合的次数
#define plainThreshold1 0.05
#define plainThreshold2 0.10

float bdbox_range=2/3;
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

void equation3(float *A, float *B, float *C, float *D, float x1, float y1, float z1, float x2, float y2, float z2,  float x3, float y3, float z3)
{
    //equation2(x, y, a1 * c2 - a2 * c1, b1 * c2 - b2 * c1, d1 * c2 - d2 * c1, a1 * c3 - a3 * c1, b1 * c3 - b3 * c1, d1 * c3 - d3 * c1);
    *A = (y2 - y1) * (z3 - z1)- (z2 - z1) * (y3 - y1);
    *B = (z2 - z1) * (x3 - x1)- (x2 - x1) * (z3 - z1);
    *C = (x2 - x1) * (y3 - y1)- (y2 - y1) * (x3 - x1);
    *D = -x1 * (y2 - y1) * (z3 - z1)- z1 * (x2 - x1) * (y3 - y1) - y1 * (z2 - z1) * (x3 - x1) + z1 * (y2 - y1) * (x3 - x1) +y1 * (x2 - x1) * (z3 - z1) + x1 * (z2 - z1) * (y3 - y1);

}

void estimate(vector<Point3f> &data,vector<float> &parameters)
{

    parameters.clear();
    if(data.size()<3)
        return;

    float A, B, C, D;
    float x1, y1, z1;
    float x2, y2, z2;
    float x3, y3, z3;

    x1 = data[0].x;
    y1 = data[0].y;
    z1 = data[0].z;

    x2 = data[1].x;
    y2 = data[1].y;
    z2 = data[1].z;

    x3 = data[2].x;
    y3 = data[2].y;
    z3 = data[2].z;

    //equation3(&A, &B, &C, &D,);

    equation3(&A, &B, &C, &D, x1, y1, z1, x2, y2, z2, x3, y3, z3);

    float norm_A,norm_B,norm_C,norm_D;
    float mod;

    if (abs(D)<0.01)
    {
        norm_D=0.00;
        mod=sqrt(A*A+B*B+C*C);
        norm_A=A/mod;
        norm_B=B/mod;
        norm_C=C/mod;
    }
    else
    {
        norm_D=1.00;
        mod=sqrt((A/D)*(A/D)+(B/D)*(B/D)+(C/D)*(C/D));
        norm_A=(A/D)/mod;
        norm_B=(B/D)/mod;
        norm_C=(C/D)/mod;
    }

    parameters.push_back(norm_A);
    parameters.push_back(norm_B);
    parameters.push_back(norm_C);
    parameters.push_back(norm_D);

    /*parameters.push_back(data[0]->x);
    parameters.push_back(data[0]->y); */
}


bool agree1(unsigned char &label, vector<float> &parameters, Point3f &data)
{
    int sizeParam=parameters.size();
    float signedDistance = parameters[sizeParam-4]*data.x+parameters[sizeParam-3]*data.y+parameters[sizeParam-2]*data.z+parameters[sizeParam-1];
    if (label==16)
        return (abs(signedDistance) < plainThreshold2);
    return (abs(signedDistance) < plainThreshold1);
}

void myRansac(unsigned char &label, int &cntSave1, vector<Point3f> &data, int &left, int &right, int &top, int &bottom, vector<float> &parameters1,cv::Mat depth_image,vector<float> &parameters_get1)
{
    int cntInlier=0;
    bool verify_flag;
    vector<Point3f> data_plainFit;
    Point3f p_all;
    float paramA1, paramB1, paramC1, paramD1;

    data_plainFit.clear();
    data_plainFit.push_back(data[0]);
    data_plainFit.push_back(data[1]);
    data_plainFit.push_back(data[2]);

    estimate(data_plainFit,parameters1);

//    float verticalA=parameters1[0];
//    float verticalB=parameters1[1];
//    float verticalC=parameters1[2];
    //if (fabs(verticalC)>=0.4&&fabs(verticalA)>=0.2)//parameters1.end()
    //if (fabs(verticalB)>=0.7)//parameters1.end()
    //{
        for (int k=top+(bottom-top)*bdbox_range;k<bottom;k++)//行号for (int k=220;k<424;k++)
        {
            for (int j=left;j<right;j++)//列号for (int j=50;j<462;j++)
            {
                PointType pt1;
                if ( depth_image.at<unsigned short>(k, j) == 0 ) {
                    pt1.x = 0;
                    pt1.y = 0;
                    pt1.z = 0;
                    continue;
                }
                else
                {
                    pt1.z = depth_image.at<unsigned short>(k, j)/1000.0;
                    pt1.y = (k-cy)*pt1.z/f;
                    pt1.x = (j-cx)*pt1.z/f;

                    p_all=Point3f(pt1.x, pt1.y, pt1.z);
                    verify_flag=agree1(label,parameters1,p_all);
                    if (verify_flag==1)
                    {
                        cntInlier++;
                    }
                }
            }
        }

        if (cntInlier>cntSave1)
        {
            cntSave1=cntInlier;
            paramA1=parameters1[0];
            paramB1=parameters1[1];
            paramC1=parameters1[2];
            paramD1=parameters1[3];
            parameters_get1.push_back(paramA1);
            parameters_get1.push_back(paramB1);
            parameters_get1.push_back(paramC1);
            parameters_get1.push_back(paramD1);
        }

        return;
}

pcl::PointCloud<PointType> dem_plain(cv::Mat depth_image,unsigned char label, int &left, int &right, int &top, int &bottom, double& bond_left, double& bond_right, double& bond_top, double& bond_bottom)//if there is not walkable  return 3.
{
    float dist1,dist2,dist3,dist1_x,dist1_y,dist1_z,dist2_x,dist2_y,dist2_z,dist3_x,dist3_y,dist3_z;
    dist1=1.00;
    dist2=2.00;
    dist3=3.00;
    pcl::PointCloud<PointType> pointCloud;

    vector<Point3f> point_selected;
    vector<float> parameters1;
    vector<float> parameters_get1;
    int cntSave1=0;

//    cv::Point centerpoint;
//    unsigned char* ptr;
//    for (int pit=0;pit!=cloud_filtered->size();pit++)
//    {
//        centerpoint.x=int(cloud_filtered->points[pit].x*f/cloud_filtered->points[pit].z+cx);
//        centerpoint.y=int(cloud_filtered->points[pit].y*f/cloud_filtered->points[pit].z+cy);
//        ptr=_binImg.data+centerpoint.y*_binImg.step + centerpoint.x * _binImg.elemSize();
//        ptr[0]=int(label)*10;
//    }

//    if ((label==5)||(label==9)||(label==12)||(label==16))
//    {
//        left-=30;
//        right+=30;
//        top-=30;
//        bottom+=50;
//    }

//    if (label==16)
//    {

//        int aaa=0;
//        for (double k=top;k<bottom;k++)//行号for (double k=5;k<424;k++)
//        {
////            if ((k>top+(bottom-top)*2/3)&&(!flag_ok))
////            {
////                left+=30;
////                right-=30;
////                flag_ok=true;
////            }
//            for (double j=left;j<right;j++)//列号for (double j=5;j<467;j++)
//            {
//                vector<float> point;
//                PointType pt;
//                if ((depth_image.at<unsigned short>(k, j)==0)||(depth_image.at<unsigned short>(k, j)>3600.0))
//                {
//                    pt.x = std::numeric_limits<float>::quiet_NaN();
//                    pt.y = std::numeric_limits<float>::quiet_NaN();
//                    pt.z = std::numeric_limits<float>::quiet_NaN();
//                    pointCloud.points.push_back( pt );
//                    continue;
//                }
//                else
//                {
//                    pt.z = depth_image.at<unsigned short>(k, j)/1000.0;
//                    pt.y = (k-cy)*pt.z/f;
//                    pt.x = (j-cx)*pt.z/f;
//                }
//                if (label==2)
//                {
//                    pt.r = 255;
//                    pt.g = 0;
//                    pt.b = 0;
//                }
//                else if (label==5)
//                {
//                    pt.r = 255;
//                    pt.g = 255;
//                    pt.b = 0;
//                }
//                else if (label==9)
//                {
//                    pt.r = 0;
//                    pt.g = 255;
//                    pt.b = 0;
//                }
//                else if (label==12)
//                {
//                    pt.r = 0;
//                    pt.g = 255;
//                    pt.b = 255;
//                }
//                else if (label==15)
//                {
//                    pt.r = 0;
//                    pt.g = 0;
//                    pt.b = 255;
//                }
//                else if (label==16)
//                {
//                    pt.r = 160;
//                    pt.g = 32;
//                    pt.b = 240;
//                }
//                else
//                {
//                    pt.r = 0;
//                    pt.g = 0;
//                    pt.b = 0;
//                }

//                pt.label = label;

//                if (aaa==0)
//                {
//                    bond_left=640;
//                    bond_right=0;
//                    bond_top=480;
//                    bond_bottom=0;
//                    aaa++;
//                }
//                else
//                {
//                    if (j<bond_left)
//                    {
//                        bond_left=j;
//                    }
//                    if (j>bond_right)
//                    {
//                        bond_right=j;
//                    }
//                    if (k<bond_top)
//                    {
//                        bond_top=k;
//                    }
//                    if (k>bond_bottom)
//                    {
//                        bond_bottom=k;
//                    }
//                }
//                pointCloud.points.push_back( pt );
//            }
//        }


//    pointCloud.width  = right-left;
//    pointCloud.height = bottom-top;
//    pointCloud.is_dense = true;//false;

//    return pointCloud;
//    }

    if ((label==15))
    {
        left-=70;
        right-=100;
//                top-=30;
                bottom-=60;
    }
            if ((label==16))
            {
                left+=100;
                right+=100;
//                top-=30;
                bottom+=30;
            }
    srand((unsigned)time(0));
    bool flag_found=true;

    int bdbox_range_fan=(bottom-top)*(1-bdbox_range);
    for (int fittimes=0;fittimes<MAX_ITERATOR;fittimes++)
    {
        point_selected.clear();
        parameters1.clear();
        //在image_3d中随机取三个点，并保证这四个点都不是空点（0,0,0）
        int randrow1=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
        int randcol1=rand()%(right-left)+left;
        int randrow2=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
        int randcol2=rand()%(right-left)+left;
        int randrow3=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
        int randcol3=rand()%(right-left)+left;

        PointType pt1;
        if ( depth_image.at<unsigned short>(randrow1, randcol1) == 0 ) {
            pt1.x = 0.0f;
            pt1.y = 0.0f;
            pt1.z = 0.0f;
        }
        else
        {
            pt1.z = depth_image.at<unsigned short>(randrow1, randcol1)/1000.0f;
            pt1.y = (randrow1-cy)*pt1.z/f;
            pt1.x = (randcol1-cx)*pt1.z/f;
        }

        PointType pt2;
        if ( depth_image.at<unsigned short>(randrow2, randcol2) == 0 ) {
            pt2.x = 0.0f;
            pt2.y = 0.0f;
            pt2.z = 0.0f;
        }
        else
        {
            pt2.z = depth_image.at<unsigned short>(randrow2, randcol2)/1000.0f;
            pt2.y = (randrow2-cy)*pt2.z/f;
            pt2.x = (randcol2-cx)*pt2.z/f;
        }

        PointType pt3;
        if ( depth_image.at<unsigned short>(randrow3, randcol3) == 0 ) {
            pt3.x = 0.0f;
            pt3.y = 0.0f;
            pt3.z = 0.0f;
        }
        else
        {
            pt3.z = depth_image.at<unsigned short>(randrow3, randcol3)/1000.0f;
            pt3.y = (randrow3-cy)*pt3.z/f;
            pt3.x = (randcol3-cx)*pt3.z/f;
        }

        vector<float> point1;
        vector<float> point2;
        vector<float> point3;

        point1.push_back(pt1.x);
        point1.push_back(pt1.y);
        point1.push_back(pt1.z);

//        cout<<"Point2:  "<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<endl;
        point2.push_back(pt2.x);
        point2.push_back(pt2.y);
        point2.push_back(pt2.z);

//        cout<<"Point3:  "<<pt3.x<<" "<<pt3.y<<" "<<pt3.z<<endl;
        point3.push_back(pt3.x);
        point3.push_back(pt3.y);
        point3.push_back(pt3.z);

        int chongfu=0;
        flag_found=true;
        while ((point1[0]==0)||(point2[0]==0)||(point3[0]==0)||(dist1+dist2-dist3<0.001)||(dist1-dist2+dist3<0.001)||(dist2+dist3-dist1<0.001))
        {
            chongfu++;
            if (chongfu==1000)
            {
                flag_found=false;
                break;
            }
            if ((point1[0]==0)||(point2[0]==0)||(point3[0]==0))
            {
                if (point1[0]==0)
                {
                    randrow1=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol1=rand()%(right-left)+left;
                    PointType pt1;
                    if ( depth_image.at<unsigned short>(randrow1, randcol1) == 0 ) {
                        pt1.x = 0;
                        pt1.y = 0;
                        pt1.z = 0;
                    }
                    else
                    {
                        pt1.z = depth_image.at<unsigned short>(randrow1, randcol1)/1000.0;
                        pt1.y = (randrow1-cy)*pt1.z/f;
                        pt1.x = (randcol1-cx)*pt1.z/f;
                    }
                    point1[0]=pt1.x;
                    point1[1]=pt1.y;
                    point1[2]=pt1.z;

                }
                if (point2[0]==0)
                {
                    randrow2=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol2=rand()%(right-left)+left;

                    PointType pt2;
                    if ( depth_image.at<unsigned short>(randrow2, randcol2) == 0 ) {
                        pt2.x = 0;
                        pt2.y = 0;
                        pt2.z = 0;
                    }
                    else
                    {
                        pt2.z = depth_image.at<unsigned short>(randrow2, randcol2)/1000.0;
                        pt2.y = (randrow2-cy)*pt2.z/f;
                        pt2.x = (randcol2-cx)*pt2.z/f;
                    }
                    point2[0]=pt2.x;
                    point2[1]=pt2.y;
                    point2[2]=pt2.z;

                }
                if (point3[0]==0)
                {
                    randrow3=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol3=rand()%(right-left)+left;

                    PointType pt3;
                    if ( depth_image.at<unsigned short>(randrow3, randcol3) == 0 ) {
                        pt3.x = 0;
                        pt3.y = 0;
                        pt3.z = 0;
                    }
                    else
                    {
                        pt3.z = depth_image.at<unsigned short>(randrow3, randcol3)/1000.0;
                        pt3.y = (randrow3-cy)*pt3.z/f;
                        pt3.x = (randcol3-cx)*pt3.z/f;
                    }
                    point3[0]=pt3.x;
                    point3[1]=pt3.y;
                    point3[2]=pt3.z;

                }
                dist1=1.00;
                dist2=2.00;
                dist3=3.00;
            }
            else
            {
                //判断是否三点共线

                dist1_x=point1[0]-point2[0];
                dist1_y=point1[1]-point2[1];
                dist1_z=point1[2]-point2[2];
                dist2_x=point2[0]-point3[0];
                dist2_y=point2[1]-point3[1];
                dist2_z=point2[2]-point3[2];
                dist3_x=point3[0]-point1[0];
                dist3_y=point3[1]-point1[1];
                dist3_z=point3[2]-point1[2];

                dist1=sqrt(dist1_x*dist1_x+dist1_y*dist1_y+dist1_z*dist1_z);
                dist2=sqrt(dist2_x*dist2_x+dist2_y*dist2_y+dist2_z*dist2_z);
                dist3=sqrt(dist3_x*dist3_x+dist3_y*dist3_y+dist3_z*dist3_z);
                if ((dist1+dist2-dist3<0.001)||(dist1-dist2+dist3<0.001)||(dist2+dist3-dist1<0.001))
                {
                    randrow1=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol1=rand()%(right-left)+left;
                    PointType pt1;
                    if ( depth_image.at<unsigned short>(randcol1, randrow1) == 0 ) {
                        pt1.x = 0;
                        pt1.y = 0;
                        pt1.z = 0;
                    }
                    else
                    {
                        pt1.z = depth_image.at<unsigned short>(randcol1, randrow1)/1000.0;
                        pt1.y = (randrow1-cy)*pt1.z/f;
                        pt1.x = (randcol1-cx)*pt1.z/f;
                    }

                    point1[0]=pt1.x;
                    point1[1]=pt1.y;
                    point1[2]=pt1.z;

                    randrow2=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol2=rand()%(right-left)+left;

                    PointType pt2;
                    if ( depth_image.at<unsigned short>(randcol2, randrow2) == 0 ) {
                        pt2.x = 0;
                        pt2.y = 0;
                        pt2.z = 0;
                    }
                    else
                    {
                        pt2.z = depth_image.at<unsigned short>(randcol2, randrow2)/1000.0;
                        pt2.y = (randrow2-cy)*pt2.z/f;
                        pt2.x = (randcol2-cx)*pt2.z/f;
                    }
                    point2[0]=pt2.x;
                    point2[1]=pt2.y;
                    point2[2]=pt2.z;

                    randrow3=rand()%bdbox_range_fan+top+(bottom-top)*bdbox_range;//取得[270,423]的随机整数
                    randcol3=rand()%(right-left)+left;

                    PointType pt3;
                    if ( depth_image.at<unsigned short>(randcol3, randrow3) == 0 ) {
                        pt3.x = 0;
                        pt3.y = 0;
                        pt3.z = 0;
                    }
                    else
                    {
                        pt3.z = depth_image.at<unsigned short>(randcol3, randrow3)/1000.0;
                        pt3.y = (randrow3-cy)*pt3.z/f;
                        pt3.x = (randcol3-cx)*pt3.z/f;
                    }
                    point3[0]=pt3.x;
                    point3[1]=pt3.y;
                    point3[2]=pt3.z;

                    dist1=1.00;
                    dist2=2.00;
                    dist3=3.00;
                }
                else
                {
                    flag_found=true;
                    break;
                }

            }
        }

        if (flag_found==false)
        {
            break;
        }

        point_selected.clear();
        point_selected.push_back(Point3f(point1[0], point1[1], point1[2]));
        point_selected.push_back(Point3f(point2[0], point2[1], point2[2]));
        point_selected.push_back(Point3f(point3[0], point3[1], point3[2]));

        myRansac(label,cntSave1,point_selected,left,right,top,bottom,parameters1,depth_image,parameters_get1);
        parameters1.clear();
    }

    if (flag_found==false)
    {
        pointCloud.width  = 0;
        pointCloud.height = 0;
        pointCloud.is_dense = true;//false;
        return pointCloud;
    }

    double alpha=0.0f;
    Point3f point_main;

    //已经得到了parameters
    bool verifymain_flag;
    int sizeParam=0;
    sizeParam=parameters_get1.size();
//    cout<<"here........."<<parameters_get1[sizeParam-4]<<" "<<parameters_get1[sizeParam-3]<<" "<<parameters_get1[sizeParam-2]<<" "<<parameters_get1[sizeParam-1]<<endl;
//    if (parameters_get1[sizeParam-1]==parameters_get2[sizeParam-1])
//    {
//        alpha=(parameters_get1[sizeParam-4]*parameters_get2[sizeParam-4]+parameters_get1[sizeParam-3]*parameters_get2[sizeParam-3]+parameters_get1[sizeParam-2]*parameters_get2[sizeParam-2]);
//    }
//    else
//    {
//        alpha=0;
//    }


    if ((label==15))
    {
        left+=70;
        right+=100;
//                top+=30;
                bottom+=60;
    }
    if ((label==16))
    {
        left-=100;
        right-=100;
//                top-=30;
        bottom-=30;
    }

    std::cout<<"sizeParam........"<<sizeParam<<std::endl;
    if (sizeParam<4)
    {
        alpha=0;
    }
    else
    {
        alpha=parameters_get1[sizeParam-3];
    }

//    std::cout<<parameters_get1[sizeParam-4]<<" "<<parameters_get1[sizeParam-3]<<" "<<parameters_get1[sizeParam-2]<<" "<<parameters_get1[sizeParam-1]<<std::endl;

    bool flag_ok=false;
    int aaa=0;

    std::cout<<"alpha..............."<<alpha<<std::endl;

    if ((alpha>angleThreshold)||((alpha<-angleThreshold)))
    {
        for (double k=top;k<bottom;k++)//行号for (double k=5;k<424;k++)
        {
//            if ((k>top+(bottom-top-30)*2/3)&&(!flag_ok))
//            {
//                left+=30;
//                right-=30;
//                flag_ok=true;
//            }
            for (double j=left;j<right;j++)//列号for (double j=5;j<467;j++)
            {
                vector<float> point;
                PointType pt;
                if ((depth_image.at<unsigned short>(k, j)==0)||(depth_image.at<unsigned short>(k, j)>3600.0))
                {
                    pt.x = std::numeric_limits<float>::quiet_NaN();
                    pt.y = std::numeric_limits<float>::quiet_NaN();
                    pt.z = std::numeric_limits<float>::quiet_NaN();
                    pointCloud.points.push_back( pt );
                    continue;
                }
                else
                {
                    pt.z = depth_image.at<unsigned short>(k, j)/1000.0;
                    pt.y = (k-cy)*pt.z/f;
                    pt.x = (j-cx)*pt.z/f;
                }
                point.clear();
                point.push_back(pt.x);
                point.push_back(pt.y);
                point.push_back(pt.z);

                point_main=Point3f(point[0], point[1], point[2]);

                verifymain_flag=agree1(label,parameters_get1,point_main);
                if (verifymain_flag!=1)
                {
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

                    if (aaa==0)
                    {
                        bond_left=640;
                        bond_right=0;
                        bond_top=480;
                        bond_bottom=0;
                        aaa++;
                    }
                    else
                    {
                        if (j<bond_left)
                        {
                            bond_left=j;
                        }
                        if (j>bond_right)
                        {
                            bond_right=j;
                        }
                        if (k<bond_top)
                        {
                            bond_top=k;
                        }
                        if (k>bond_bottom)
                        {
                            bond_bottom=k;
                        }
                    }
                    pointCloud.points.push_back( pt );
                }
            }
        }
    }
    else
    {
//        bottom-=15;
        bool flag_ok=false;
//        if (label==9)
//            bottom=bottom-(bottom-top)/4;
//        else if (label==15)
//            bottom=bottom-(bottom-top)/9;
        for (double k=top;k<bottom;k++)//行号for (double k=5;k<424;k++)
        {
//            if ((k>top+(bottom-top)*2/3)&&(!flag_ok))
//            {
//                left+=30;
//                right-=30;
//                flag_ok=true;
//            }
            for (double j=left;j<right;j++)//列号for (double j=5;j<467;j++)
            {
                vector<float> point;
                PointType pt;
                if ((depth_image.at<unsigned short>(k, j)==0)||(depth_image.at<unsigned short>(k, j)>3600.0))
                {
                    pt.x = std::numeric_limits<float>::quiet_NaN();
                    pt.y = std::numeric_limits<float>::quiet_NaN();
                    pt.z = std::numeric_limits<float>::quiet_NaN();
                    pointCloud.points.push_back( pt );
                    continue;
                }
                else
                {
                    pt.z = depth_image.at<unsigned short>(k, j)/1000.0;
                    pt.y = (k-cy)*pt.z/f;
                    pt.x = (j-cx)*pt.z/f;
                }
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

                pt.label = label;

                if (aaa==0)
                {
                    bond_left=640;
                    bond_right=0;
                    bond_top=480;
                    bond_bottom=0;
                    aaa++;
                }
                else
                {
                    if (j<bond_left)
                    {
                        bond_left=j;
                    }
                    if (j>bond_right)
                    {
                        bond_right=j;
                    }
                    if (k<bond_top)
                    {
                        bond_top=k;
                    }
                    if (k>bond_bottom)
                    {
                        bond_bottom=k;
                    }
                }
                pointCloud.points.push_back( pt );
            }
        }
    }


    pointCloud.width  = right-left;
    pointCloud.height = bottom-top;
    pointCloud.is_dense = true;//false;

    return pointCloud;
}

pcl::PointCloud<PointType> depth2cloud( cv::Mat depth_image, /*cv::Mat rgb_image*/unsigned char label, int left, int right, int top, int bottom, double& bond_left, double& bond_right, double& bond_top, double& bond_bottom) {

    pcl::PointCloud<PointType> pointCloud;
    int aaa=0;

    pointCloud=dem_plain(depth_image, label, left, right, top, bottom, bond_left, bond_right, bond_top, bond_bottom);

//        for ( int x = left; x < right; ++ x ) {
//            for ( int y = top; y < bottom; ++ y ) {
//                PointType pt;
//                if ( depth_image.at<unsigned short>(y, x) == 0 ) {
//                    pt.x = std::numeric_limits<float>::quiet_NaN();
//                    pt.y = std::numeric_limits<float>::quiet_NaN();
//                    pt.z = std::numeric_limits<float>::quiet_NaN();
//                }
//                else
//                {
//                    pt.z = depth_image.at<unsigned short>(y, x)/1000.0;
//                    pt.y = (y-cy)*pt.z/f;
//                    pt.x = (x-cx)*pt.z/f;

//                    if (label==2)
//                    {
//                        pt.r = 255;
//                        pt.g = 0;
//                        pt.b = 0;
//                    }
//                    else if (label==5)
//                    {
//                        pt.r = 255;
//                        pt.g = 255;
//                        pt.b = 0;
//                    }
//                    else if (label==9)
//                    {
//                        pt.r = 0;
//                        pt.g = 255;
//                        pt.b = 0;
//                    }
//                    else if (label==12)
//                    {
//                        pt.r = 0;
//                        pt.g = 255;
//                        pt.b = 255;
//                    }
//                    else if (label==15)
//                    {
//                        pt.r = 0;
//                        pt.g = 0;
//                        pt.b = 255;
//                    }
//                    else if (label==16)
//                    {
//                        pt.r = 160;
//                        pt.g = 32;
//                        pt.b = 240;
//                    }
//                    else
//                    {
//                        pt.r = 0;
//                        pt.g = 0;
//                        pt.b = 0;
//                    }
////                    pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
////                    pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
////                    pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
//                    pt.label = label;
//                }
//                if (aaa==0)
//                {
//                    bond_left=pt.x;
//                    bond_right=pt.x;
//                    bond_top=pt.y;
//                    bond_bottom=pt.y;
//                    aaa++;
//                }
//                else
//                {
//                    if (pt.x<bond_left)
//                    {
//                        bond_left=pt.x;
//                    }
//                    if (pt.x>bond_right)
//                    {
//                        bond_right=pt.x;
//                    }
//                    if (pt.y<bond_top)
//                    {
//                        bond_top=pt.y;
//                    }
//                    if (pt.y>bond_bottom)
//                    {
//                        bond_bottom=pt.y;
//                    }

//                }

//                pointCloud.points.push_back( pt );
//            }
//        }

//        pointCloud.width  = right-left;
//        pointCloud.height = bottom-top;
//        pointCloud.is_dense = true;//false;

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
