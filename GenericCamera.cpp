/* 
 * File:   GenericCamera.cpp
 * Author: hans
 * 
 * Created on 3 de Junho de 2014, 13:11
 */

#include "GenericCamera.h"

using namespace std;

GenericCamera::GenericCamera() {
    init();
};

bool GenericCamera::open(const int cap){
    opened = true;
    return true;
}

bool GenericCamera::configure() {
    return true;
}

bool GenericCamera::start(){ 
    started = true;
    return true;    
}

void GenericCamera::autoFocus() {
    
}

void GenericCamera::stop(){ 
    started = false;
}

void GenericCamera::close(){ 
    opened = false;
}

bool GenericCamera::isOpened()  const{ 
    return opened;    
}

bool GenericCamera::isStarted()  const{ 
    return started;
}

cv::Mat *GenericCamera::convert() {
   
    return imageBuffer;
}

cv::Mat *GenericCamera::grabbImage(bool raw){ 
    return NULL;
}

bool GenericCamera::saveRawFrame(const std::string &filename, bool raw) {
   
    for (int i=0; i<MAX_GRAB_TRIALS; i++) {
        cv::Mat *frame = grabbImage(raw);
        if (frame!=NULL) {
            if (cv::imwrite(filename, *frame)) {
                return true;
            }
            else {
                cout << "Could not write image to file: "  << filename << endl;
                return false;
            }
        }
    }
    
    cout << "Could not grabb image to save. " << endl;
    return false;
}

int GenericCamera::getRoiMax(int x0, int y0, int width, int height) {
    //Make a rectangle
    //Point a cv::Mat header at it (no allocation is done)
    cv::Mat *frame = grabbImage(true);
    
/////////Corrigir o roi: isso deveria ser feito na geração do roi e não aqui
    if (width<0) {
        x0 = x0 + width;
        width = -width;
    }
    if (height<0) {
        y0 = y0 + height;
        height = -height;
    }

    if (x0<0) {
        width-=x0;
        x0=0;
    }
    if (x0>frame->cols-1) x0=frame->cols-1;
    
    if (y0<0) {
        height-=y0;
        y0=0;
    }
    
    if (y0>frame->rows-1) y0=frame->rows-1;
    if (x0+width>frame->cols) width = frame->cols - x0;
    if (y0+height>frame->rows) height = frame->rows - y0;
/////////
    
    cv::Rect roirect(x0, y0, width, height);

    cv::Mat imageRoi = (*frame)(roirect);

    double min, max;
    cv::minMaxLoc(imageRoi, &min, &max);
    
    return max;
}

GenericCamera::~GenericCamera() {
    
    if (outBuffer!=NULL)
        delete outBuffer;
}

void GenericCamera::init()
{
    imgType=0;
    outputType=0;
    width=0;
    height=0;
    opened=false;
    started=false;
    imageBuffer=NULL;
    outBuffer=NULL;
}