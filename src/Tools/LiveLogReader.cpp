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

#include "LiveLogReader.h"

unsigned short unCharToUnShort(unsigned char* pBuf)
{
 unsigned short result = 0;
 result = (short)pBuf[0]*256;
 result += (short)pBuf[1];
 return result;
}

LiveLogReader::LiveLogReader(std::string file, bool flipColors)
 : LogReader(file, flipColors),
   lastFrameTime(-1),
   lastGot(-1)
{
    std::cout << "Creating live capture... "; std::cout.flush();

	asus = new OpenNI2Interface(Resolution::getInstance().width(), Resolution::getInstance().height());

    decompressionBufferDepth = new Bytef[Resolution::getInstance().numPixels() * 2];

	decompressionBufferImage = new Bytef[Resolution::getInstance().numPixels() * 3];

//    if(!asus->ok())
//    {
//        std::cout << "failed!" << std::endl;
//        std::cout << asus->error();
//    }
//    else
//    {
        std::cout << "success!" << std::endl;

        std::cout << "Waiting for first frame"; std::cout.flush();

//        int lastDepth = asus->latestDepthIndex.getValue();

        int lastDepth =1;
//        do
//        {
//            usleep(33333);
//            std::cout << "."; std::cout.flush();
//            lastDepth = asus->latestDepthIndex.getValue();
//        } while(lastDepth == -1);

        std::cout << " got it!" << std::endl;
//    }
}

LiveLogReader::~LiveLogReader()
{
    delete [] decompressionBufferDepth;

    delete [] decompressionBufferImage;

	delete asus;
}

void LiveLogReader::getNext(cv::Mat &image_in, cv::Mat &depth_in/*, Eigen::Matrix4f tran*/, const bool flag_slam,int count_global)
{
    int lastDepth = asus->latestDepthIndex.getValue();

    assert(lastDepth != -1);

    int bufferIndex = lastDepth % OpenNI2Interface::numBuffers;

    if(bufferIndex == lastGot)
    {
        return;
    }

    if(lastFrameTime == asus->frameBuffers[bufferIndex].second)
    {
        return;
    }

//    memcpy(&decompressionBufferDepth[0], asus->frameBuffers[bufferIndex].first.first, Resolution::getInstance().numPixels() * 2);
//    memcpy(&decompressionBufferImage[0], asus->frameBuffers[bufferIndex].first.second, Resolution::getInstance().numPixels() * 3);

//    memset(&decompressionBufferDepth[0], 0, Resolution::getInstance().numPixels() * 2);
//    memset(&decompressionBufferImage[0], 0, Resolution::getInstance().numPixels() * 3);
    std::cout<<depth_in.data[0]<<std::endl;
    memcpy((unsigned short*)&decompressionBufferDepth[0], /*(unsigned short *)*/depth_in.data, Resolution::getInstance().numPixels() * 2);
    memcpy(&decompressionBufferImage[0], image_in.data, Resolution::getInstance().numPixels() * 3);

    lastFrameTime = asus->frameBuffers[bufferIndex].second;

    timestamp = lastFrameTime;

//    double* ptr = (double*) depth_in.data;
//    size_t elem_step = depth_in.step / sizeof(double);
//    double val = ptr[i * elem_step + j];

//    std::cout<<depth_in.step<<std::endl;
//    for (int y = 0; y < depth_in.rows; ++y)
//    {
//        unsigned short* row_ptr = depth_in.ptr<unsigned short>(y);
//        for (int x = 0; x < depth_in.cols; ++x)
//        {
//            unsigned short val = row_ptr[x];
//            if (val<0.01)
//                row_ptr[x]=0;
//                std::cout<<row_ptr[x]<<std::endl;
//        }
//    }

    rgb = (unsigned char *)&decompressionBufferImage[0];
    depth = (unsigned short *)&decompressionBufferDepth[0];

    imageReadBuffer = 0;
    depthReadBuffer = 0;

    imageSize = Resolution::getInstance().numPixels() * 3;
    depthSize = Resolution::getInstance().numPixels() * 2;

    if(flipColors)
    {
        for(int i = 0; i < Resolution::getInstance().numPixels() * 3; i += 3)
        {
            std::swap(rgb[i + 0], rgb[i + 2]);
        }
    }
}

const std::string LiveLogReader::getFile()
{
    return Parse::get().baseDir().append("live");
}

int LiveLogReader::getNumFrames()
{
    return std::numeric_limits<int>::max();
}

bool LiveLogReader::hasMore()
{
    return true;
}

void LiveLogReader::setAuto(bool value)
{
    asus->setAutoExposure(value);
    asus->setAutoWhiteBalance(value);
}
