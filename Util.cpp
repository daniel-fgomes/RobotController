#include "Util.h"

double getCurrentRealTimer(void)
{
    struct timeval t;
    gettimeofday(&t, 0);
    return t.tv_sec + t.tv_usec * 1.0e-6;
}

double getCurrentCPUTimer(void)
{
	return static_cast<double>(clock()) / CLOCKS_PER_SEC;
}

const char* getTimeStamp(const char sp)
{
    static char buf[16];
    
    time_t rawtime;
    struct tm *timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    sprintf(buf,"%d%c%d",((timeinfo->tm_year - 100)*100 + (timeinfo->tm_mon + 1))*100 + timeinfo->tm_mday,sp,(timeinfo->tm_hour * 100 + timeinfo->tm_min)*100 + timeinfo->tm_sec);
    
    return buf;
}
