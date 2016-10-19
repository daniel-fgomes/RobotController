/*
 * This software is governed by the LGPL v3.0 license. 
 * By downloading, copying, installing or using the software you agree to
 * this license. If you do not agree to this license, do not download, 
 * install, copy or use the software.
*/

/* 
 * File:   config.h
 * Author: daniel
 *
 * Created on November 2, 2014, 12:32 PM
 * 
 * Arquivo com as diversas posibilidades de compilação
 */


#ifndef CTVSTUBROBOTMODULECONFIG_H
#define	CTVSTUBROBOTMODULECONFIG_H
#include <iostream>


#define TESTCONTROL
//#define TESTMOVE
//#define TESTODOMETER
//#define TESTLINK
#define CONTROL_SERIALPORT ("/dev/ttyUSB0")

//#define NOWEBCAM
#define WEBCAMNUMBER 1
#define VERBOSEMODE 1

#define ERRMSG(TXT) { std::cerr << TXT << " file:" << __FILE__ << " line:" << __LINE__ << std::endl; }
#define MSG(TXT) { std::cout << TXT << " file:" << __FILE__ << " line:" <<  __LINE__ << std::endl; }

#endif	/* CTVSTUBROBOTMODULECONFIG_H */

