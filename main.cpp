
/*
 * This software is governed by the LGPL v3.0 license. 
 * By downloading, copying, installing or using the software you agree to
 * this license. If you do not agree to this license, do not download, 
 * install, copy or use the software.
*/

#include "RobotModuleConfig.h"
#include "RobotController.h"
#include <stdio.h>

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>

#define ROBOTID_NOANSWER 0
#define ROBOTID_SUCCESS 1
#define ROBOTID_FAILURE 2

#define ROBOTID_JOYSTICK 0
#define ROBOTID_AUTO 1

struct termios initial_settings,
new_settings;


int main(int argc, char **argv) {

    std::cout << " Inicializando o ... " << std::endl;

#ifdef TESTCONTROL 


    using namespace std;
    RobotController robot;
    robot.init();

    unsigned char key;



    tcgetattr(0, &initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);

    
    robot.inputImg();
    uchar u;

    robot.clearOdometer();
    for (int i = 0; i < 100000; i++) {

        robot.moveAutoTrack(0.9);
        cv::imshow("teste", *robot.img);
        
        u = cv::waitKey(1);

        usleep(10000);
        u = getchar();

        if (u == 27)
            break;
    }


    robot.disconnect();
#endif
    
    return 0;
}
