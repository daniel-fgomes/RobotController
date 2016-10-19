/* 
 * File:   Webcam.cpp
 * Author: Hans
 * 
 * Created on 13 de Junho de 2014, 13:56
 */

#include "Webcam.h"


Webcam::Webcam(TColorGray color) {
    init();
    this->color = color;
    imageBuffer = new cv::Mat();
    outBuffer = NULL;
    capture=NULL;
    
    if (color == WC_GRAY)
        outputType = CV_8UC1;
    else
        outputType = CV_8UC3;
}

bool Webcam::open(const int cap){
    
    if (isOpened()) return true;
    
    capture = new cv::VideoCapture(cap);
    
    if (capture->isOpened()) {
        opened = true;
        return true;
    }
    return false;
}

bool Webcam::configure() {
    return true;
}

bool Webcam::start(){
    if (isOpened() && !isStarted()) {
        for (int i=0; i<MAX_GRAB_TRIALS; i++) {
            if (capture->read(*imageBuffer)) {
                width = imageBuffer->cols;
                height = imageBuffer->rows;
                started = true;
            }
        } 
    }
    
    return started;
}


void Webcam::stop(){
    started = false;
}

void Webcam::close(){ 
    if(capture!=NULL)
    if (capture->isOpened())
    {
        if(started) stop();
        capture->release();
    }
    opened = false;
}

 cv::Mat *Webcam::grabbImage(bool raw) {
     
     if (!isStarted()) return NULL;
     
     if (capture->read(*imageBuffer)) {
         
        if (raw)
           return imageBuffer;
        else
           return convert();
     }
     
     return NULL;
 }

cv::Mat *Webcam::convert() {
    if (color == WC_GRAY) {
        if (outBuffer == NULL)
            outBuffer = new cv::Mat(imageBuffer->rows, imageBuffer->cols, CV_8UC1);

        cv::cvtColor(*imageBuffer, *outBuffer, CV_BGR2GRAY);
        return outBuffer;
    } else
        if(color == WC_COLOR) {
        if (outBuffer == NULL)
            outBuffer = new cv::Mat(imageBuffer->rows, imageBuffer->cols, CV_8UC1);

        cv::cvtColor(*imageBuffer, *outBuffer, CV_BGR2RGB);
        return outBuffer;
    }

    return imageBuffer;
}

void Webcam::setParam(const int prop, const int v) {
    if (capture) {
        capture->set(prop,v);
    }
}

Webcam::~Webcam() {
    
    if (imageBuffer!=NULL)
        delete imageBuffer;
}

