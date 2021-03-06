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
 
#include "../../rangeimage/include/rangeimage/MainController.h"
#include "../../rangeimage/include/rangeimage/rangeimage.h"

void saveColorData(GLbyte* colorArr, std::string& _str)
{
    cv::Mat img;
    vector<cv::Mat> imgPlanes;
    img.create(Resolution::getInstance().height(),Resolution::getInstance().width(),CV_8UC3);
    cv::split(img,imgPlanes);

//    FILE* pFile = NULL;
//    pFile = fopen(_str.c_str(), "wt");
//    if(!pFile) { fprintf(stderr, "error \n"); exit(-1); }
    for(int i=0; i<Resolution::getInstance().width() * Resolution::getInstance().height() * 3; i ++) {
    if(colorArr[i] == -1) { colorArr[i] = 255; }
    }
    for(int i = 0; i < Resolution::getInstance().height(); i ++)
    {
        unsigned char* plane0Ptr = imgPlanes[0].ptr<unsigned char>(i);
        unsigned char* plane1Ptr = imgPlanes[1].ptr<unsigned char>(i);
        unsigned char* plane2Ptr = imgPlanes[2].ptr<unsigned char>(i);
        for(int j = 0; j < Resolution::getInstance().width(); j ++)
        {
            int k = 3 * (i * Resolution::getInstance().width() + j); // RGBA
            plane2Ptr[j] = colorArr[k];
            plane1Ptr[j] = colorArr[k+1];
            plane0Ptr[j] = colorArr[k+2];
        }
    }

    cv::merge(imgPlanes, img);
    cv::imwrite(_str.c_str(), img);
}

MainController::MainController()
 : good(true),
   eFusion(0),
   gui(0),
   groundTruthOdometry(0),
   logReader(0),
   framesToSkip(0),
   resetButton(false),
   resizeStream(0)
{
    int argc=1;
    char** argv;
    std::string empty;
    iclnuim = Parse::get().arg(argc, argv, "-icl", empty) > -1;

    std::string calibrationFile;
    Parse::get().arg(argc, argv, "-cal", calibrationFile);

    Resolution::getInstance(640, 480);
//    Resolution::getInstance(672, 376);

    if(calibrationFile.length())
    {
        loadCalibration(calibrationFile);
    }
    else
    {
       Intrinsics::getInstance(528, 528, 320, 240);
//         Intrinsics::getInstance(528, 528, 336, 188);
    }

    Parse::get().arg(argc, argv, "-l", logFile);

//    if(logFile.length())
//    {
        logReader = new RawLogReader(logFile, Parse::get().arg(argc, argv, "-f", empty) > -1);
//    }
//    else
//    {
//        logReader = new LiveLogReader(logFile, Parse::get().arg(argc, argv, "-f", empty) > -1);

//        good = ((LiveLogReader *)logReader)->asus->ok();
//    }

    if(Parse::get().arg(argc, argv, "-p", poseFile) > 0)
    {
        groundTruthOdometry = new GroundTruthOdometry(poseFile);
    }

    confidence = 5.0f;
    depth = 10.0f;
    icp = 10.0f;
    icpErrThresh = 5e-05;
    covThresh = 1e-05;
    photoThresh = 115;
    fernThresh = 0.3095f;

    timeDelta = 200;
    icpCountThresh = 35000;
    start = 1;

//    confidence = 2.0f;//10.0f;
//    depth = 3.0f;
//    icp = 10.0f;
//    icpErrThresh = 5e-05;
//    covThresh = 1e-05;
//    photoThresh = 115;
//    fernThresh = 0.3095f;

//    timeDelta = 200;
//    icpCountThresh = 35000;
//    start = 1;
//    so3 = !(Parse::get().arg(argc, argv, "-nso", empty) > -1);
    end = std::numeric_limits<unsigned short>::max(); //Funny bound, since we predict times in this format really!

    Parse::get().arg(argc, argv, "-c", confidence);
    Parse::get().arg(argc, argv, "-d", depth);
    Parse::get().arg(argc, argv, "-i", icp);
    Parse::get().arg(argc, argv, "-ie", icpErrThresh);
    Parse::get().arg(argc, argv, "-cv", covThresh);
    Parse::get().arg(argc, argv, "-pt", photoThresh);
    Parse::get().arg(argc, argv, "-ft", fernThresh);
    Parse::get().arg(argc, argv, "-t", timeDelta);
    Parse::get().arg(argc, argv, "-ic", icpCountThresh);
    Parse::get().arg(argc, argv, "-s", start);
    Parse::get().arg(argc, argv, "-e", end);

    logReader->flipColors = true;//Parse::get().arg(/*argc, argv,*/ "-f", empty) > -1;

    openLoop = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1;
    reloc = Parse::get().arg(argc, argv, "-rl", empty) > -1;
    frameskip = Parse::get().arg(argc, argv, "-fs", empty) > -1;
    quiet = Parse::get().arg(argc, argv, "-q", empty) > -1;
    fastOdom = Parse::get().arg(argc, argv, "-fo", empty) > -1;
    rewind = Parse::get().arg(argc, argv, "-r", empty) > -1;
    frameToFrameRGB = Parse::get().arg(argc, argv, "-ftf", empty) > -1;

    gui = new GUI(logFile.length() == 0, Parse::get().arg(argc, argv, "-sc", empty) > -1);

    gui->flipColors->Ref().Set(logReader->flipColors);
    gui->rgbOnly->Ref().Set(false);
    gui->pyramid->Ref().Set(true);
    gui->fastOdom->Ref().Set(fastOdom);
    gui->confidenceThreshold->Ref().Set(confidence);
    gui->depthCutoff->Ref().Set(depth);
    gui->icpWeight->Ref().Set(icp);
    gui->so3->Ref().Set(so3);
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);

    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);
}

MainController::~MainController()
{
    if(eFusion)
    {
        delete eFusion;
    }

    if(gui)
    {
        delete gui;
    }

    if(groundTruthOdometry)
    {
        delete groundTruthOdometry;
    }

    if(logReader)
    {
        delete logReader;
    }

    if(resizeStream)
    {
        delete resizeStream;
    }
}

void MainController::loadCalibration(const std::string & filename)
{
    std::ifstream file(filename);
    std::string line;

    assert(!file.eof());

    double fx, fy, cx, cy;

    std::getline(file, line);

    int n = sscanf(line.c_str(), "%lg %lg %lg %lg", &fx, &fy, &cx, &cy);

    assert(n == 4 && "Ooops, your calibration file should contain a single line with fx fy cx cy!");

    Intrinsics::getInstance(fx, fy, cx, cy);
}

void MainController::initializeeFusion()
{
    eFusion=new ElasticFusion;
}

void MainController::launch(const int& index_file_rgb, int &dir_num, cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg/*, Eigen::Matrix4f tran*/, const bool flag_slam, int64_t count_global)
{
//    while(good)
//    {
        if(eFusion)
        {
            run(index_file_rgb, dir_num,image_in, depth_in, labelImg/*, tran*/, flag_slam, count_global);
        }

//        if(eFusion == 0 || resetButton)
//        {
//            resetButton = false;

//            if(eFusion)
//            {
//                delete eFusion;
//            }

//            logReader->rewind();
//            eFusion = new ElasticFusion(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
//                                        icpCountThresh,
//                                        icpErrThresh,
//                                        covThresh,
//                                        !openLoop,
//                                        iclnuim,
//                                        reloc,
//                                        photoThresh,
//                                        confidence,
//                                        depth,
//                                        icp,
//                                        fastOdom,
//                                        fernThresh,
//                                        so3,
//                                        frameToFrameRGB,
//                                        logReader->getFile());
//        }
////        else
////        {
////            break;
////        }

////    }
}

void MainController::run(const int& index_file_rgb, int &dir_num, cv::Mat &image_in, cv::Mat &depth_in, cv::Mat &labelImg/*, Eigen::Matrix4f& tran*/, const bool flag_slam, int64_t count_global)
{
//    while(!pangolin::ShouldQuit() && !((!logReader->hasMore()) && quiet) && !(eFusion->getTick() == end && quiet))
//    {
        if(!gui->pause->Get() || pangolin::Pushed(*gui->step))
        {
            if((logReader->hasMore() || rewind) && eFusion->getTick() < end)
            {
                TICK("LogRead");
                if(rewind)
                {
                    if(!logReader->hasMore())
                    {
                        logReader->getBack(image_in, depth_in, labelImg/*, tran*/, flag_slam,count_global);
//                        count_global++;
                    }
                    else
                    {
                        logReader->getNext(image_in, depth_in, labelImg/*, tran*/, flag_slam,count_global);
//                        count_global++;
                    }

                    if(logReader->rewound())
                    {
                        logReader->currentFrame = 0;
                    }
                }
                else
                {
                    logReader->getNext(image_in, depth_in, labelImg/*, tran*/, flag_slam,count_global);
//                    count_global++;
                }
                TOCK("LogRead");

                if(eFusion->getTick() < start)
                {
                    eFusion->setTick(start);
                    logReader->fastForward(start);
                }

                float weightMultiplier = framesToSkip + 1;

                if(framesToSkip > 0)
                {
                    eFusion->setTick(eFusion->getTick() + framesToSkip);
                    logReader->fastForward(logReader->currentFrame + framesToSkip);
                    framesToSkip = 0;
                }

                Eigen::Matrix4f * currentPose = 0;//&tran;

//                if(groundTruthOdometry)
//                {
//                    currentPose = new Eigen::Matrix4f;
//                    currentPose->setIdentity();
//                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
//                }

//                tran=tran.inverse();
                eFusion->processFrame(count_global, flag_slam, logReader->rgb, logReader->depth, logReader->label_ef, logReader->timestamp, currentPose/*&tran*/, weightMultiplier);

//                if(currentPose)
//                {
//                    delete currentPose;
//                }

                if(frameskip && Stopwatch::getInstance().getTimings().at("Run") > 1000.f / 30.f)
                {
                    framesToSkip = int(Stopwatch::getInstance().getTimings().at("Run") / (1000.f / 30.f));
                }
            }
        }
        else
        {
            eFusion->predict();
        }

        TICK("GUI");

        if(gui->followPose->Get())
        {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f forwardVector(0, 0, 1);
            Eigen::Vector3f upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();  // Forward
            Eigen::Vector3f x = up.cross(z).normalized(); // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m << x(0),  x(1),  x(2),  -(x.dot(eye)),
                 y(0),  y(1),  y(2),  -(y.dot(eye)),
                 z(0),  z(1),  z(2),  -(z.dot(eye)),
                    0,     0,     0,              1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if(!gui->pause->Get())
        {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if(gui->drawRawCloud->Get() || gui->drawFilteredCloud->Get())
        {
            eFusion->computeFeedbackBuffers();
        }

        if(gui->drawRawCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::RAW)->render(flag_slam, gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawFilteredCloud->Get())
        {
            eFusion->getFeedbackBuffers().at(FeedbackBuffer::FILTERED)->render(flag_slam, gui->s_cam.GetProjectionModelViewMatrix(), pose, gui->drawNormals->Get(), gui->drawColors->Get());
        }

        if(gui->drawGlobalModel->Get())
        {
            glFinish();
            TICK("Global");

            if(gui->drawFxaa->Get())
            {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            }
            else
            {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if(eFusion->getLost())
        {
            glColor3f(1, 1, 0);
        }
        else
        {
            glColor3f(1, 0, 1);
        }
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if(gui->drawFerns->Get())
        {
            glColor3f(0, 0, 0);
            for(size_t i = 0; i < eFusion->getFerns().frames.size(); i++)
            {
                if((int)i == eFusion->getFerns().lastClosest)
                    continue;

                gui->drawFrustum(eFusion->getFerns().frames.at(i)->pose);
            }
            glColor3f(1, 1, 1);
        }

        if(gui->drawDefGraph->Get())
        {
            const std::vector<GraphNode*> & graph = eFusion->getLocalDeformation().getGraph();

            for(size_t i = 0; i < graph.size(); i++)
            {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for(size_t j = 0; j < graph.at(i)->neighbours.size(); j++)
                {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if(eFusion->getFerns().lastClosest != -1)
        {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff)
            {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for(size_t i = 0; i < poseMatches.size(); i++)
        {
            if(gui->drawDeforms->Get())
            {
                if(poseMatches.at(i).fern)
                {
                    glColor3f(1, 0, 0);
                }
                else
                {
                    glColor3f(0, 1, 0);
                }
                for(size_t j = 0; j < poseMatches.at(i).constraints.size(); j++)
                {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for(std::map<std::string, GPUTexture*>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it)
        {
            if((it->second->draw)&&(it->first!="LABEL"))
            {
                gui->displayImg(it->first, it->second);
            }
        }

        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        std::string tupian1="/media/disk1/xr/Now/data_klg/";
        tupian1+=int2str(dir_num);
        tupian1+="/res/ef_out1/";
        tupian1 += int2str(index_file_rgb);
        tupian1 += ".png";

        std::string tupian2="/media/disk1/xr/Now/data_klg/";
        tupian2+=int2str(dir_num);
        tupian2+="/res/ef_out_global1/";
        tupian2 += int2str(index_file_rgb);
        tupian2 += ".png";

        GLbyte* colorArr = new GLbyte[3* Resolution::getInstance().width() * Resolution::getInstance().height()];
        \
        eFusion->getIndexMap().imageTex()->texture->Download(colorArr, GL_RGB, GL_UNSIGNED_BYTE);

        saveColorData(colorArr,tupian1);
        delete colorArr;
        colorArr=0;

        colorArr = new GLbyte[3* Resolution::getInstance().width() * Resolution::getInstance().height()];
        \
        eFusion->indexMap.combinedPredict(eFusion->currPose,
                                        eFusion->globalModel.model(),
                                        eFusion->maxDepthProcessed,
                                        eFusion->confidenceThreshold,
                                        0,
                                        eFusion->tick,
                                        eFusion->timeDelta,
                                        IndexMap::ACTIVE);

        eFusion->getIndexMap().imageTex()->texture->Download(colorArr, GL_RGB, GL_UNSIGNED_BYTE);

        saveColorData(colorArr,tupian2);
        delete colorArr;
        colorArr=0;

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();

        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();

        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();

        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();

        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();

        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();

        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if(gui->autoSettings)
        {
            static bool last = gui->autoSettings->Get();

            if(gui->autoSettings->Get() != last)
            {
                last = gui->autoSettings->Get();
                static_cast<LiveLogReader *>(logReader)->setAuto(last);
            }
        }

        Stopwatch::getInstance().sendAll();

//        if(resetButton)
//        {
//            break;
//        }

        if(pangolin::Pushed(*gui->save))
        {
            eFusion->savePly();
        }

        TOCK("GUI");
//    }

//    count_global++;
}
