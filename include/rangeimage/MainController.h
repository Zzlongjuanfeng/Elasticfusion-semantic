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

#include "../../src/ElasticFusion.h"
#include "../../src/Utils/Parse.h"

#include "../../src/Tools/GUI.h"
#include "../../src/Tools/GroundTruthOdometry.h"
#include "../../src/Tools/RawLogReader.h"
#include "../../src/Tools/LiveLogReader.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

//#include "rangeimage/rangeimage.h"

#ifndef MAINCONTROLLER_H_
#define MAINCONTROLLER_H_

class MainController
{
    public:
        MainController();
        virtual ~MainController();

        void initializeeFusion();
        void launch(const int &index_file_rgb, int &dir_num, cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg/*, Eigen::Matrix4f tran*/, const bool flag_slam, int64_t count_global);

        void run(const int &index_file_rgb, int &dir_num, cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg, const bool flag_slam, int64_t count_global);
    private:
        void loadCalibration(const std::string & filename);
        ElasticFusion *eFusion;
        bool good;
        GUI * gui;
        GroundTruthOdometry * groundTruthOdometry;
        LogReader * logReader;

        bool iclnuim;
        std::string logFile;
        std::string poseFile;

        float confidence,
              depth,
              icp,
              icpErrThresh,
              covThresh,
              photoThresh,
              fernThresh;

        int timeDelta,
            icpCountThresh,
            start,
            end;

        bool fillIn,
             openLoop,
             reloc,
             frameskip,
             quiet,
             fastOdom,
             so3,
             rewind,
             frameToFrameRGB;

        int framesToSkip;
        bool streaming;
        bool resetButton;

        Resize * resizeStream;
};

#endif /* MAINCONTROLLER_H_ */
