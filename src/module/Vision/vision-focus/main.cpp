/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/all.h>


#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "closestBlob_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > {
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > cropOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> triggerPort;
    yarp::os::Port classifPort;

    yarp::os::Port SFM3dpoint;

    yarp::os::RpcClient rpc;
    yarp::os::Mutex mutex;


    //Processing parameters
    int gausian_size;
    double cannyParam;
    double coefTresh;
    int dilate_niter;
    int erode_iter;

    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;

    const std::string labels [2] = {"mug","sodaBottle"};

    yarp::dev::IGazeControl *igaze = NULL;
    std::vector<double> finalPosition;

    std::vector<cv::Rect> boundingRects;
    std::vector<cv::Point> matchRectCog;


public:


    /********************************************************/

    Processing(const std::string &moduleName) {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing() {

    };

    /********************************************************/
    bool open() {

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open("/" + moduleName + "/disparity:i");
        inPort.open("/" + moduleName + "/image:i");
        outPort.open("/" + moduleName + "/image:o");
        cropOutPort.open("/" + moduleName + "/crop:o");
        targetPort.open("/" + moduleName + "/target:o");
        triggerPort.open("/" + moduleName + "/trigger:i");
        classifPort.open("/" + moduleName + "/classif:i");
        SFM3dpoint.open("/" + moduleName + "/2dpoint:o");

        gausian_size = 9;
        cannyParam = 60;
        coefTresh = 0.7;
        dilate_niter = 20;
        erode_iter = 30;

	/*	
        yarp::os::Property option;
        option.put("device", "gazecontrollerclient");
        option.put("remote", "/iKinGazeCtrl");
        option.put("local", "/client/gaze");

        yarp::dev::PolyDriver clientGazeCtrl(option);

        if (clientGazeCtrl.isValid()) {
            clientGazeCtrl.view(igaze);
        }

	*/


        return true;
    }

    /********************************************************/
    void close() {
        inPort.close();
        outPort.close();
        targetPort.close();
        cropOutPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
    }

    /********************************************************/
    void interrupt() {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
    }

    /********************************************************/
    std::vector<double> getPoint(const int32_t index) {


        if(matchRectCog.size() > 0 ){
            finalPosition.clear();
            yarp::os::Bottle classifB;

            yarp::os::Bottle Point2D, Point3D;

            //yarp::os::Bottle *trigger = triggerPort.read();
            yarp::os::Bottle &outTargets = targetPort.prepare();


            outTargets.clear();
            outTargets.addString(labels[index]);

            if (outTargets.size() > 0) {

                targetPort.write();
            }



            classifPort.read(classifB);
            int matchIndex = classifB.get(0).asInt();

            if (matchRectCog.size() < 2) {
                matchIndex = 0;
            }


            yInfo() << "Matching rect is " << matchRectCog[matchIndex].x << " " << matchRectCog[matchIndex].y;
            yarp::sig::Vector px(2);
            px[0] = matchRectCog[matchIndex].x;
            px[1] = matchRectCog[matchIndex].y;

            //const std::string cmd = "Root " + matchRectCog[matchIndex].x + matchRectCog[matchIndex].y;
        
            Point2D.clear();
                
            Point2D.addString("Root");
            Point2D.addInt(matchRectCog[matchIndex].x);
            Point2D.addInt(matchRectCog[matchIndex].y);
            SFM3dpoint.write(Point2D, Point3D);

            yInfo() << "3D is " << Point3D.toString();

            finalPosition.push_back(Point3D.get(0).asDouble());
            finalPosition.push_back(Point3D.get(1).asDouble());
            finalPosition.push_back(Point3D.get(2).asDouble());

            matchRectCog.clear();

        }

        else{
            finalPosition.push_back(0);
            finalPosition.push_back(0);
            finalPosition.push_back(0);
        }

        return finalPosition;

    }


    /********************************************************/
    void onRead(yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage) {

        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage = outPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage = cropOutPort.prepare();


        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();


        outImage.resize(dispImage.width(), dispImage.height());
        cropOutImage.resize(dispImage.width(), dispImage.height());

        outImage.zero();
        cropOutImage.zero();

        cv::Mat inColour_cv = cv::cvarrToMat(
                (IplImage *) inImage->getIplImage());  // prepare the image ports and targets
        cv::Mat outZeros_cv = cv::cvarrToMat(
                (IplImage *) cropOutImage.getIplImage());  // prepare the image ports and targets
        cv::Mat disp = cv::cvarrToMat((IplImage *) dispImage.getIplImage());


        // Apply image processing techniques on the disparity image to smooth things out
        mutex.lock();
        cv::GaussianBlur(disp, disp, cv::Size(gausian_size, gausian_size), 2, 2);
        mutex.unlock();

        cv::Mat copyDisp = disp.clone();
        // Find the max value and its position
        cv::minMaxLoc(disp, &minVal, &maxVal, &minLoc, &maxLoc);


        // Apply some threshold on the image to remove background
        // have a look at cv::threshold function
        mutex.lock();
        cv::threshold(disp, copyDisp, maxVal * coefTresh, 0, 4);
        mutex.unlock();


        //Find the contour of the closest objects with moments and mass center

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat canny;

        /// Detect edges using canny
        mutex.lock();
        cv::Canny(copyDisp, canny, cannyParam, cannyParam * 3, 3);
        mutex.unlock();

        //void dilate(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue()
        mutex.lock();
        cv::dilate(disp, disp, cv::Mat(), cv::Point(-1,-1), dilate_niter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
        mutex.unlock();

        //void erode(InputArray src, OutputArray dst, InputArray kernel, Point anchor=Point(-1,-1), int iterations=1, int borderType=BORDER_CONSTANT, const Scalar& borderValue=morphologyDefaultBorderValue() )
        mutex.lock();
        cv::erode(disp, disp, cv::Mat(), cv::Point(-1,-1), erode_iter, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
        mutex.unlock();



        cv::findContours(canny, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_TC89_L1);


        cvtColor(disp, disp, CV_GRAY2RGB);
        cv::Rect boundRect;
        bool findContours = false;
        std::vector<cv::Point> contours_poly;

        cv::Mat cropImage;

        // Use the result of pointPolygonTest or your own technique as the closest contour to:

        cv::Scalar color = cv::Scalar(0, 200, 200);
        cv::Scalar colorM = cv::Scalar(0, 255, 0);


        int i = 0;
        int j = 0;
        while (i < contours.size() && j < 2) {
            if (pointPolygonTest(contours[i], maxLoc, true)) {
                //yInfo() << "Find contours";

                cv::approxPolyDP(cv::Mat(contours[i]), contours_poly, 3, true);
                boundRect = boundingRect(cv::Mat(contours_poly));
                boundingRects.push_back(boundRect);

                cv::drawContours(disp, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                cv::Moments mu = cv::moments(contours_poly);
                cv::Point2f momentsCoord = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);

                cv::circle(disp, momentsCoord, 4, colorM, -1, 8, 0);
                findContours = true;


                const int x = boundRect.tl().x + (boundRect.width / 2);
                const int y = boundRect.tl().y + (boundRect.height / 2);

                matchRectCog.push_back(cv::Point(x, y));
                ++j;

            }
            ++i;
        }




        IplImage out = disp;
        outImage.resize(out.width, out.height);
        cvCopy(&out, (IplImage *) outImage.getIplImage());
        outPort.write();

        if (findContours) {
            //yInfo() << "Crop Image";

            for(int i = 0; i < boundingRects.size() ; ++i){

                cropImage = inColour_cv(boundingRects[i]);
                cropImage.copyTo(outZeros_cv(boundingRects[i]));
            }

        }




        IplImage crop = outZeros_cv;
        cropOutImage.resize(crop.width, crop.height);
        cvCopy(&crop, (IplImage *) cropOutImage.getIplImage());
        cropOutPort.write();


        boundingRects.clear();

    }


    /********************************************************/
    bool setGausianSize(const int32_t size) {
        mutex.lock();
        gausian_size = size;
        mutex.unlock();
        return true;
    }


    /********************************************************/
    bool setCoefThreshold(const double t_coef) {
        mutex.lock();
        coefTresh = t_coef;
        mutex.unlock();
        return true;
    }

    /********************************************************/
    bool setCannyParam(const int32_t t_canny) {
        mutex.lock();
        cannyParam = t_canny;
        mutex.unlock();
        return true;
    }

    /********************************************************/
    bool setErodeParam(const int32_t t_e) {
        mutex.lock();
        erode_iter = t_e;
        mutex.unlock();
        return true;
    }

    /********************************************************/
    bool setDilateParam(const int32_t t_di) {
        mutex.lock();
        dilate_niter = t_di;
        mutex.unlock();
        return true;
    }


    /********************************************************/
    int32_t getGausianSize() {
        return gausian_size;
    }

    /********************************************************/
    double getCoefThreshold() {
        return coefTresh;
    }

    /********************************************************/
    int32_t getCannyParam() {
        return cannyParam;
    }

    /********************************************************/
    int32_t getErodeParam() {
        return erode_iter;
    }
    /********************************************************/
    int32_t getDilateParam() {
        return dilate_niter;
    }


};

/********************************************************/
class Module : public yarp::os::RFModule, public closestBlob_IDL {
    yarp::os::ResourceFinder *rf;
    yarp::os::RpcServer rpcPort;

    Processing *processing;

    friend class processing;

    bool closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source) {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf) {
        this->rf = &rf;
        std::string moduleName = rf.check("name", yarp::os::Value("closest-blob"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/robot/detector/rpc"));

        closing = false;

        processing = new Processing(moduleName);

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close() {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit() {
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod() {
        return 0.1;
    }

    /********************************************************/
    bool updateModule() {
        return !closing;
    }


    /********************************************************/
    bool setCannyparam(const int32_t iter) {
        processing->setCannyParam(iter);
        return true;
    }

    /********************************************************/
    bool setCoefThreshold(const double iter) {
        processing->setCoefThreshold(iter);
        return true;
    }

    /********************************************************/
    bool setErodeParam(const int32_t t_e) {
        processing->setErodeParam(t_e);
        return true;
    }

    /********************************************************/
    bool setDilateParam(const int32_t t_di) {
        processing->setDilateParam(t_di);
        return true;
    }

    /********************************************************/
    bool setGausianSize(const int32_t size) {
        bool success = true;
        if (size & 0x01)
            processing->setGausianSize(size);
        else
            success = false;

        return success;
    }


    /********************************************************/
    int32_t getCannyParam() {
        return processing->getCannyParam();
    }

    /********************************************************/
    int32_t getErodeParam() {
        return processing->getErodeParam();
    }
    /********************************************************/
    int32_t getDilateParam() {
        return processing->getDilateParam();
    }


    /********************************************************/
    double getCoefThreshold() {
        return processing->getCoefThreshold();
    }

    /********************************************************/
    int32_t getGausianSize() {
        return processing->getGausianSize();
    }

    /********************************************************/
    std::vector<double> get3DPoint(int32_t t_index){
        yInfo() << "Get labels" << t_index;
        return processing->getPoint(t_index);
    }



};

/********************************************************/
int main(int argc, char *argv[]) {
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc, argv);

    return module.runModule(rf);
}
