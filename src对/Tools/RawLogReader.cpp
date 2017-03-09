/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "RawLogReader.h"

#include <fstream>
#include <string.h>
#include <sstream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

string int2str(int n)
{
    stringstream ss;
    ss<<n;

    string s;
    ss>>s;
    return s;
}
RawLogReader::RawLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors)
{
//    assert(pangolin::FileExists(file.c_str()));

//    fp = fopen(file.c_str(), "rb");

    currentFrame = 0;

//    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    numFrames=2194;

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
    labelReadBuffer = new unsigned char[numPixels * 3];
    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];
    decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];
    decompressionBufferLabel = new Bytef[Resolution::getInstance().numPixels() * 3];
}

RawLogReader::~RawLogReader()
{
    delete [] labelReadBuffer;
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;
    delete [] decompressionBufferDepth;
    delete [] decompressionBufferImage;
    delete [] decompressionBufferLabel;

    fclose(fp);
}

void RawLogReader::getBack(cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg/*, Eigen::Matrix4f tran*/, const bool flag_slam,int64_t count_global)
{
    assert(filePointers.size() > 0);

    fseek(fp, filePointers.top(), SEEK_SET);

    filePointers.pop();

    getCore(image_in,depth_in,labelImg/*, tran*/,flag_slam,count_global);
}

void RawLogReader::getNext(cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg/*, Eigen::Matrix4f tran*/, const bool flag_slam,int64_t count_global)
{
//    filePointers.push(ftell(fp));

//    int cntmm=0;
//              for (int i=0;i<640;i++)
//                  for (int j=0;j<480;j++)
//                      if (labelImg.at<unsigned char>(i,j)!=0){
//  //                        std::cout<<labelImg.at<unsigned char>(i,j)<<std::endl;
//                          cntmm++;
//                      }
//              cout<<cntmm<<endl;

    getCore(image_in,depth_in,labelImg,flag_slam,count_global);
}

void RawLogReader::getCore(cv::Mat &rgb_in, cv::Mat &depth_in, cv::Mat &labelImg, const bool flag_slam, int64_t count_global)
{
    int64_t timeStamp;
//    int cntmm=0;
//              for (int i=0;i<640;i++)
//                  for (int j=0;j<480;j++)
//                      if (labelImg.at<unsigned char>(i,j)!=0){
//  //                        std::cout<<labelImg.at<unsigned char>(i,j)<<std::endl;
//                          cntmm++;
//                      }
//              cout<<cntmm<<endl;

//    string DataPath="/media/disk1/xr/Data_mine/1/";
//    std::string Dir(DataPath);

//    std::string  rgbname, depthname;

//    rgbname=int2str(count_global)+".jpg";
//    depthname=int2str(count_global)+".png";

//    std::string rgbDir = Dir+"rgb/"+rgbname;
//    std::string depthDir = Dir+"depth/"+depthname;

//    /*cv::Mat*/ rgb_in = cv::imread(rgbDir);
//    /*cv::Mat*/ depth_in = cv::imread(depthDir, -1);

    timeStamp=count_global;

    int depth_compress_buf_size = width * height * sizeof(int16_t) * 4;//500*97*2*4
    uint8_t * depth_compress_buf = (uint8_t*)malloc(depth_compress_buf_size);
    unsigned long compressed_size = depth_compress_buf_size;

    compress2(depth_compress_buf,
              &compressed_size,
              (const Bytef*)depth_in.data,
              width * height * sizeof(short),
              Z_BEST_SPEED);

    cv::Mat3b rgb_3(height, width, (cv::Vec<unsigned char, 3> *)rgb_in.data, width*3);
    cv::Mat3b label_3;
    labelImg.copyTo(label_3);

//    cv::Mat3b label_3(height, width, (cv::Vec<unsigned char, 3> *)labelImg.data, width*3);

//    IplImage * img = new IplImage(rgb_3);
    int jpeg_params[] = {CV_IMWRITE_JPEG_QUALITY, 90, 0};
//    CvMat * encodedImage = cvEncodeImage(".jpg", img, jpeg_params);

    depthSize = compressed_size;
    imageSize = numPixels * 3;//encodedImage->width;

//    assert(fread(&timestamp, sizeof(int64_t), 1, fp));

//    assert(fread(&depthSize, sizeof(int32_t), 1, fp));
//    assert(fread(&imageSize, sizeof(int32_t), 1, fp));

//    assert(fread(depthReadBuffer, depthSize, 1, fp));

//    memcpy(&depthReadBuffer[0], depth_compress_buf, Resolution::getInstance().numPixels() * 2);
//    if (count_global==84){
//        cout<<"stop"<<endl;
////        memcpy(&imageReadBuffer[0], encodedImage->data.ptr/*rgb_in.data*/, Resolution::getInstance().numPixels() * 3);
//    }
    if(imageSize > 0)
    {
//        assert(fread(imageReadBuffer, imageSize, 1, fp));
        memcpy(&depthReadBuffer[0], depth_compress_buf, Resolution::getInstance().numPixels() *2 );
        memcpy(&imageReadBuffer[0], /*encodedImage->data.ptr*/rgb_3.data, Resolution::getInstance().numPixels() * 3);
        memcpy(&labelReadBuffer[0], /*(Bytef*)*/label_3.data, Resolution::getInstance().numPixels()*3);
    }

    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBufferDepth[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBufferDepth[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
    }

    if(imageSize == numPixels * 3)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
        memcpy(&decompressionBufferLabel[0], labelReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        memcpy(&decompressionBufferImage[0], imageReadBuffer, numPixels * 3);
        memcpy(&decompressionBufferLabel[0], labelReadBuffer, numPixels * 3);
//        jpeg.readData(imageReadBuffer, imageSize, (unsigned char *)&decompressionBufferImage[0]);
    }
    else
    {
        memset(&decompressionBufferImage[0], 0, numPixels * 3);
        memset(&decompressionBufferLabel[0], 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBufferDepth;
    rgb = (unsigned char *)&decompressionBufferImage[0];
    label_ef=(unsigned char *)&decompressionBufferLabel[0];

    flipColors=true;
    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }

    currentFrame++;
}

void RawLogReader::fastForward(int frame)
{
    while(currentFrame < frame && hasMore())
    {
        filePointers.push(ftell(fp));

        assert(fread(&timestamp, sizeof(int64_t), 1, fp));

        assert(fread(&depthSize, sizeof(int32_t), 1, fp));
        assert(fread(&imageSize, sizeof(int32_t), 1, fp));

        assert(fread(depthReadBuffer, depthSize, 1, fp));

        if(imageSize > 0)
        {
            assert(fread(imageReadBuffer, imageSize, 1, fp));
        }

        currentFrame++;
    }
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}


void RawLogReader::rewind()
{
    if (filePointers.size() != 0)
    {
        std::stack<int> empty;
        std::swap(empty, filePointers);
    }

//    fclose(fp);
//    fp = fopen(file.c_str(), "rb");//"r"是以文本形式读,"rb"是以二进制的形式读

//    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    numFrames=2194;
    currentFrame = 0;
}

bool RawLogReader::rewound()
{
    return filePointers.size() == 0;
}

const std::string RawLogReader::getFile()
{
    return file;
}

void RawLogReader::setAuto(bool value)
{

}
