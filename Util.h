/* 
 * File:   Utilities.h
 * Author: Daniel de F.Gomes
 *
 * Created on November 29, 2012, 4:35 PM
 */

#ifndef UTIL_H
#define	UTIL_H

#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * O retorno é para um buffer de caracteres estatático
 */

const char* getTimeStamp(const char sp = '_');
#ifndef MATUTILS_H_
double getCurrentRealTimer(void);

double getCurrentCPUTimer(void);

static double ticTocElapsedTimeVar;

inline void tic(void) {
    ticTocElapsedTimeVar = getCurrentRealTimer();
};

inline double toc(void) {
    return (getCurrentRealTimer() - ticTocElapsedTimeVar);
};

//Tempo em segundos

inline void delay(double time = 1.0) {
    double start = getCurrentRealTimer();
    double delta;
    do {
        delta = getCurrentRealTimer() - start;
    } while (delta < time);
}
#endif
#endif	/* UTIL_H */

