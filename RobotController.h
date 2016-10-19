/*
 * This software is governed by the LGPL v3.0 license. 
 * By downloading, copying, installing or using the software you agree to
 * this license. If you do not agree to this license, do not download, 
 * install, copy or use the software.
*/

/* 
 * File:   RobotController.h
 * Author: daniel
 *
 * Created on October 28, 2014, 6:52 PM
 */

#ifndef ROBOTCONTROLLER_H
#define	ROBOTCONTROLLER_H

#include "RobotModuleConfig.h"
#include "TComm.h"
#include "PIDController.h"
#include "Webcam.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH_HALF (IMAGE_WIDTH/2)
#define IMAGE_HEIGHT_HALF (IMAGE_HEIGHT/2)

float calcMean(cv::Mat *img, int x, int y, int halfW, int halfH, int ch, bool draw = false) {
    if (img) {
        int x0 = x - halfW;
        int x1 = x + halfW;
        int y0 = y - halfH;
        int y1 = y + halfH;

        const int Nw = img->cols;
        const int Nh = img->rows;

        if (x0 < 0) x0 = 0;
        if (x0 >= Nw) x0 = Nw - 2;
        if (x1 < 0) x1 = 1;
        if (x1 >= Nw) x1 = Nw - 1;

        if (y0 < 0) y0 = 0;
        if (y0 >= Nh) y0 = Nh - 2;
        if (y1 < 0) y1 = 1;
        if (y1 >= Nh) y1 = Nh - 1;

        float s = 0;
        float c = 0;
        for (int i = y0; i <= y1; i++)
            for (int j = x0; j <= x1; j++) {
                s += (int) (img->at<cv::Vec3b>(i, j)[ch]);
                c++;
            }

        if (draw)
            cv::rectangle(*img, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 0, 0));

        s /= c;

        return s;
    }

    std::cerr << "NULL IMAGE!!!! File:" << __FILE__ << " line:" << __LINE__ << std::endl;
    return 0;
}

inline int findLine(cv::Mat *img, int x, int y, int halfW, int halfH, bool draw = false) {
    if (img) {
        int x0 = x - halfW;
        int x1 = x + halfW;
        int y0 = y - halfH;
        int y1 = y + halfH;

        const int Nw = img->cols;
        const int Nh = img->rows;

        if (x0 < 0) x0 = 0;
        if (x0 >= Nw) x0 = Nw - 2;
        if (x1 < 0) x1 = 1;
        if (x1 >= Nw) x1 = Nw - 1;

        if (y0 < 0) y0 = 0;
        if (y0 >= Nh) y0 = Nh - 2;
        if (y1 < 0) y1 = 1;
        if (y1 >= Nh) y1 = Nh - 1;

        float sr = 0;
        float sg = 0;
        float sb = 0;

        int i = 0;

        float c = 0;
        for (int i = y0; i <= y1; i++)
            for (int j = x0; j <= x1; j++) {
                sr += (int) (img->at<cv::Vec3b>(i, j)[2]);
                sg += (int) (img->at<cv::Vec3b>(i, j)[1]);
                sb += (int) (img->at<cv::Vec3b>(i, j)[0]);

                c++;
            }

        sr /= c;
        sg /= c;
        sb /= c;

        if (abs(sg - sr) < 45)
            if ((sg > 115) || (sr > 115))
                if (((sg + sr) / 2 - sb) > 90)
                    i = 1;

        if (draw) {
            cv::rectangle(*img, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 0, 0));
            std::cout << "R=" << sr << " G=" << sg << " B=" << sb << " L" << i << " ";
        }

        return i;
    }

    std::cerr << "NULL IMAGE!!!! File:" << __FILE__ << " line:" << __LINE__ << std::endl;
    return 0;
}

inline float fastFindLine(cv::Mat *img, int x, int y, int halfW, int halfH, bool draw = false) {
    if (img) {
        int x0 = x - halfW;
        int x1 = x + halfW;
        int y0 = y - halfH;
        int y1 = y + halfH;

        const int Nw = img->cols;
        const int Nh = img->rows;

        if (x0 < 0) x0 = 0;
        if (x0 >= Nw) x0 = Nw - 2;
        if (x1 < 0) x1 = 1;
        if (x1 >= Nw) x1 = Nw - 1;

        if (y0 < 0) y0 = 0;
        if (y0 >= Nh) y0 = Nh - 2;
        if (y1 < 0) y1 = 1;
        if (y1 >= Nh) y1 = Nh - 1;

        float sr = 0;
        float sg = 0;
        float sb = 0;

        float x = 0;

        int px = 0;

        float c = 0;
        for (int i = y0; i <= y1; i++)
            for (int j = x0; j <= x1; j += 2) {
                sr = (int) (img->at<cv::Vec3b>(i, j)[2]);
                sg = (int) (img->at<cv::Vec3b>(i, j)[1]);
                sb = (int) (img->at<cv::Vec3b>(i, j)[0]);

                px = 0;
                if (abs(sg - sr) < 45)
                    if ((sg > 115) || (sr > 115))
                        if (((sg + sr) / 2 - sb) > 90) {
                            px = 1;
                            if (draw) {
                                cv::rectangle(*img, cv::Point(j + 1, y0), cv::Point(j - 1, y1), cv::Scalar(255, 0, 0));
                                //std::cout << "R=" << sr << " G=" << sg << " B=" << sb << " L" << px << " ";
                            }
                        }


                x += j*px;
                c += px;
            }

        x /= c;

        x /= Nw;
        x = 2 * (x - 0.5);



        return x;
    }

    std::cerr << "NULL IMAGE!!!! File:" << __FILE__ << " line:" << __LINE__ << std::endl;
    return 0;
}


class RobotController {
    TComm link;
    unsigned char buf[256];
    unsigned char bufOut[256];
    TData data;
    TDataLB dataLB;
    boost::mutex mtx_;
    int trackType;
#ifndef NOWEBCAM
    Webcam webcam;
#endif

    int currentTag;
    int searchingTag;
public:
    int32_t stepL;
    int32_t stepR;

    PIDController pid;
    cv::Mat *img;

    int getIdIndex() {
        return 0;
    }

    enum {
        AUTOTRACK_OFF = 0, AUTOTRACK_X, AUTOTRACK_XY
    };

#ifndef NOWEBCAM

    RobotController() : webcam(WC_COLOR) {
#else
    RobotController(){
#endif
    }
        

        ~RobotController() {

            disconnect();
        }

        void init() {
            configPID();
            connect();

            
            autotrack(RobotController::AUTOTRACK_X);
            img = NULL;
            trackType = AUTOTRACK_OFF;
            stepL = 0;
            stepR = 0;
            currentTag = 0;

        }

        void configPID() {
            pid.setParam(1.2, 0.00, 0.0, 0, -1);
        }

        void connect() {

            if (!link.isConnected()) {
                for (int i = 0; i < 256; ++i) buf[i] = 0;

                link.open();
                sleep(2);
            }
        }

        RobotController & clearOdometer() {
        }
        
        void updateOdometer() {
        }

        float getOdometer(bool update = false) {

        }

        void disconnect() {
            if (link.isConnected()) {
                data.v = 0;
                link.write(0, data.v);
                //            link.write(1, data.v);
                link.close();
            }

#ifndef NOWEBCAM
            webcam.close();
#endif
        }

        bool isConnected() {
            return link.isConnected();
        }



        /*
         * Move o robô de considerando as restrições do tipo de navegação/tracking utilizado
         * @return retorna o tipo de navegação configurada atualmente (AUTOTRACK_OFF,AUTOTRACK_X,AUTOTRACK_XY)
         */
        int move(const float x, const float y) {
            //    boost::lock_guard<boost::mutex> guard(mtx_);

            if (link.isConnected()) {
                int ma, mb;
                char dirA = 0, dirB = 0;

                ma = -255 * x - 255 * y;
                mb = 255 * x - 255 * y;


                if (ma < 0) {
                    ma = -ma;
                    dirA = 0x80;
                } else
                    dirA = 0;

                ma = ma >> 1;
                if (ma > 127) ma = 127;

                if (mb < 0) {
                    mb = -mb;
                    dirB = 0x80;
                } else
                    dirB = 0;

                mb = mb >> 1;
                if (mb > 127) mb = 127;

                data.byte[0] = ma | dirA;
                data.byte[1] = mb | dirB;

                link.write(0, data.v);


                updateOdometer();

                //std::cerr << "x=" << x << " y=" << y << " ma=" << ma << " mb=" << mb << " odoA=" << odoA << " odoB=" << odoB << std::endl;
            }
        }

        int set(const int cmd, const int value) {

            return 0;
        }

        int moveAutoTrack(const float yy) {

            inputImg();
            float y = yy;
            float xx;
            static float x;
            xx = pid.input(trackLine()); //Auto X
            if (xx != xx) {
                y = 0;
                if (x > 0) x = 0.4;
                else x = -0.4;
            } else {
                x = xx;
            }

            move(x, y);

            return 0;
        }

        /*
         * Testa a rotina de autotrack e verifica se a linha foi detectada
         * @return retorna AUTOTRACK_X se a linha for detectada na imagem e AUTOTRACK_OFF se o autotracking não for possível
         */
        const int testAutotrack() {
            bool ret;
            float t = trackLine();
#ifndef NOWEBCAM
            ret = webcam.isOpened()&&(t <= 1)&&(t >= -1);
#endif
            return ret;
        }

        void inputImg() {
#ifndef NOWEBCAM
            using namespace std;
            if (webcam.isOpened()) {
                img = webcam.grabbImage(true);
            } else {
                ERRMSG("CÂMERA NÃO ATIVADA!! ");
                img = NULL;
            }
#endif
        }

        void inputImg(cv::Mat * img_) {
            img = img_;
        }

        /*
         * Rotina que retorna a referência da posição central da linha de track.
         * @return valor entre -1 e 1 que informa a posição da linha em relação ao centro da imagem
         */
        const float trackLine() {
            using namespace std;
            if (img != NULL) {

                float x = fastFindLine(img, IMAGE_WIDTH_HALF, IMAGE_HEIGHT_HALF, IMAGE_WIDTH_HALF - 10, 10, true);
                return x;

            } else
                ERRMSG("imagem = NULL");

            return 0;
        }

        /*
         * Modifica o modo de navegação do robô de completamente manual para completamente automático
         * @return retorna o tipo de navegação configurada atualmente (AUTOTRACK_OFF,AUTOTRACK_X,AUTOTRACK_XY)
         */
        const int autotrack(const int type = AUTOTRACK_X) {

            if (trackType != type) {

                trackType = type;

                if ((type == AUTOTRACK_X) || (type == AUTOTRACK_XY)) {
#ifndef NOWEBCAM
                    webcam.open(WEBCAMNUMBER);
                    webcam.setParam(CV_CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
                    webcam.setParam(CV_CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
                    webcam.start();
#endif
                } else {
                    trackType = AUTOTRACK_OFF;
#ifndef NOWEBCAM
                    webcam.close();
#endif
                }
            }

            return trackType;
        }



    };

#endif	/* ROBOTCONTROLLER_H */

