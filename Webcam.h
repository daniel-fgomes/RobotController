/* 
 * File:   Webcam.h
 * Author: hans
 *
 * Created on 13 de Junho de 2014, 13:56
 */

#ifndef WEBCAM_H
#define	WEBCAM_H

#include "GenericCamera.h"

typedef enum {WC_COLOR, WC_GRAY} TColorGray;

class Webcam: public GenericCamera {
public:
    Webcam(TColorGray color=WC_GRAY);
    
    virtual bool open(const int cap=0);
    virtual bool configure();
    virtual bool start();
    virtual void stop();
    virtual cv::Mat *grabbImage(bool raw = false);    
    virtual cv::Mat *convert();
    virtual void close();
    virtual void setParam(const int prop, const int v);
    
    virtual ~Webcam();
private:
    cv::VideoCapture *capture;
    TColorGray color;
};

#endif	/* WEBCAM_H */

