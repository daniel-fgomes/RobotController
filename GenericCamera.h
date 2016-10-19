/* 
 * File:   GenericCamera.h
 * Author: hans
 *
 * Created on 3 de Junho de 2014, 13:11
 */

#ifndef GENERICCAMERA_H
#define	GENERICCAMERA_H

#include <opencv2/opencv.hpp>
#include <cv.h>

#define MAX_GRAB_TRIALS 200

class GenericCamera {
public:
    GenericCamera();
    virtual ~GenericCamera();
    
    virtual bool open(const int cap=0);
    virtual bool configure();
    virtual void autoFocus();
    virtual bool start();
    virtual void stop();
    virtual cv::Mat *grabbImage(bool raw = false);    
    virtual cv::Mat *convert();
    virtual void close();

    int getImgType() {return imgType;};
    int getOutputType() {return outputType;};    
    int getWidth() {return width;};
    int getHeight() {return height;};
    bool isStarted() const;
    bool isOpened() const;    
    bool saveRawFrame(const std::string &filename, bool raw = false);
    int getRoiMax(int x0, int y0, int width, int height);
    
    
protected:
    void init();
    
    int imgType;
    int outputType;
    int width;
    int height;
    bool opened;
    bool started;
    cv::Mat *imageBuffer;
    cv::Mat *outBuffer;
};

#endif	/* GENERICCAMERA_H */

