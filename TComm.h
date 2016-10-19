/*
 * This software is governed by the LGPL v3.0 license. 
 * By downloading, copying, installing or using the software you agree to
 * this license. If you do not agree to this license, do not download, 
 * install, copy or use the software.
*/

/* 
 * File:   TComm.h
 * Author: daniel
 *
 * Created on April 19, 2014, 8:35 PM
 */

#ifndef TCOMM_H
#define	TCOMM_H
#include "RobotModuleConfig.h"
#include "SerialPort.h"
#include <stdlib.h>
#include <string>
#include <iostream>
#include <ostream>
#include "Util.h"

#define START_READ 0xF3
#define START_WRITE 0xF1
#define START_PING 'p'
#define START_RET_CMD_OK 0xF7
#define START_RET_CMD_FAIL 0xF9
#define START_RET_CMD_TIMEOUT 0xFB
#define ADDR_0 0
#define ADDR_N 10
#define TIMEOUT 2000

//Definiço do protocolo de comunicaço
//Solicitaço de leitura
//START_READ
//ADDR

//START_READ
//ADDR
//DATA1
//DATA0

//Solicitaço de escrita
//START_WRITE
//ADDR
//DATA1
//DATA0

//Respostas
//Resposta comando recebido para um write
//START_RET_CMD_OK

union TData {
    unsigned char byte[2];
    unsigned short v;
};

union TDataLB {
    unsigned char byte[4];
    int32_t v;
};

class TComm {
protected:
    SerialPort sp;
    const static int NBuffer = 256;
    unsigned char bufIn[NBuffer];
    unsigned char bufOut[NBuffer];
    unsigned char bufInTemp[NBuffer];

    int sizeIn;
    int sizeOut;
    int sizeInTemp;

    int trys;

    int deltaT;
    double timeOut;
    double initTime;
    double currentTime;
    bool singleShot;

    virtual void init() {
        trys = 10;
        singleShot=false;
        timeOut = 0.05; //Tempo em microsegundos
        deltaT = 20000; //Tempo em microsegunndos
    };
public:

    virtual int open(const std::string port = CONTROL_SERIALPORT) {
        int ret = sp.openPort(port);

        if (ret > 0)
            sp.set(B115200);

        return ret;
    };

    virtual int close() {
        sp.closePort();
        return 0;
    };

    TComm() {
        init();
    };

    virtual ~TComm() {
        close();
    };

    virtual int write(const unsigned char addr, const unsigned char *d) {

        int t = 0;

        if (sp.isOpen()) {

            bufOut[0] = START_WRITE;
            bufOut[1] = addr;
            bufOut[2] = d[1];
            bufOut[3] = d[0];

            currentTime = initTime = getCurrentRealTimer();
            while (((currentTime - initTime) < timeOut)&&(t < trys)) {

                sp.clearBuffer();
                sp.writePort(bufOut, 4);

                if(singleShot)
                    return 0;
                
                usleep(deltaT);

                sp.readPort(bufIn, NBuffer, sizeIn);
                if ((sizeIn > 0)&&(bufIn[0] == START_RET_CMD_OK)) {
                    return t;
                }

                t++;
                currentTime = getCurrentRealTimer();
            }
        }
        return -t;
    };

    virtual int write(const unsigned char addr, unsigned short v) {
        TData x;
        x.v = v;
        return write(addr, x.byte);
    }

    virtual int read(const unsigned char addr, unsigned char *d) {

        int t = 0;

        if (sp.isOpen()) {

            using namespace std;

            bufOut[0] = START_READ;
            bufOut[1] = addr;
            int i;
            bool msgBegin;
            bool msgAddr;
            int count = 0;

            currentTime = initTime = getCurrentRealTimer();
            while (((currentTime - initTime) < timeOut)&&(t < trys)) {

                sizeIn = 0;
                msgBegin = false;
                msgAddr = false;

                sp.clearBuffer();
                sp.writePort(bufOut, 2);

                usleep(deltaT);
                bufInTemp[0] = 0;
                bufInTemp[1] = 0;
                bufInTemp[2] = 0;
                bufInTemp[3] = 0;

                bufIn[0] = 0;
                bufIn[1] = 0;
                bufIn[2] = 0;
                bufIn[3] = 0;
                count = 0;
                do {
                    sp.readPort(bufInTemp, NBuffer, sizeInTemp);
                    count++;

                    for (i = 0; i < sizeInTemp; i++) {

                        if (sizeIn == 0) {
                            if (bufInTemp[i] == START_READ) {
                                msgBegin = true;
                                sizeIn = 1;
                                bufIn[0] = bufInTemp[i];
                            } else {
                                //Houve erro na recepção, reiniciar o processo
                                sizeIn = 0;
                                msgAddr = false;
                                msgBegin = false;
                            }
                        } else
                            if (sizeIn == 1) {
                            if ((msgAddr == false)&&(bufInTemp[i] == addr)) {
                                msgAddr = true;
                                sizeIn = 2;
                                bufIn[1] = addr;
                            } else {
                                //Houve erro na recepção, reiniciar o processo
                                sizeIn = 0;
                                msgAddr = false;
                                msgBegin = false;
                                goto new_try;
                            }
                        } else
                            if (sizeIn == 2) {
                            bufIn[2] = bufInTemp[i];
                            sizeIn = 3;
                        } else
                            if (sizeIn == 3) {
                            bufIn[3] = bufInTemp[i];
                            sizeIn = 4;

                            d[0] = bufIn[3];
                            d[1] = bufIn[2];

                            return t;
                        }
                    }

                    currentTime = getCurrentRealTimer();

                } while (((currentTime - initTime) < timeOut)&&(sizeIn < 4)&&(count < 4));

new_try:
                t++;
            }

        }

        return -t;
    };

    virtual int read(const unsigned char addr, unsigned short v) {
        TData x;
        x.v = v;
        return read(addr, x.byte);
    }

    virtual TComm& syncComm(const int verboseMode = 0, const int t0 = 500, const int td = 50, const int addrTest = 7,
            const int NTest = 100,
            const int t0Sync = 20000) {
        if (sp.isOpen()) {
            using namespace std;

            if (verboseMode > 0)
                cout << "Iniciando sincronização..." << flush;

            TData dataR, dataW;
            int error = 0, count = 0, j;
            int trysW = 0, trysR = 0;

            setDeltaT(t0Sync);
            dataW.v = 0;
            error = 0;
            count = 0;

            for (int i = 0; i < 1000; i++) {
                trysW = write(addrTest, dataW.v);
                trysR = read(addrTest, dataR.byte);
                if (dataR.v != dataW.v)
                    count = 0;
                else
                    count++;
                if (verboseMode >= 3)
                    cout << i << " W(" << trysW << ")=" << dataW.v << " R(" << trysR << ")=" << dataR.v << endl;
                dataW.v++;

                if (count >= 50)
                    break;
            }

            for (j = t0; j < 50000; j += td) {
                setDeltaT(j);
                dataW.v = 0;
                error = 0;

                for (int i = 0; i < NTest; i++) {
                    write(addrTest, dataW.v);
                    read(addrTest, dataR.byte);
                    if (dataR.v != dataW.v)
                        error++;
                    if (verboseMode >= 3)
                        cout << i << " W(" << trysW << ")=" << dataW.v << " R(" << trysR << ")=" << dataR.v << endl;
                    dataW.v++;
                }

                if (verboseMode >= 2
                        ) cout << "erros(" << j << "):" << error << endl;

                if (error == 0)
                    break;
            }

            if (verboseMode > 0)
                cout << "[FIM]" << endl;

        }

        return *this;
    }

    bool isConnected() {
        return sp.isOpen();
    }

    TComm& incDeltaT(const int d = 100) {
        this->deltaT += d;

        return *this;
    }

    TComm& incXDeltaT(const float d = 1.2) {
        this->deltaT *= d;

        return *this;
    }

    TComm& setDeltaT(const int deltaT) {
        this->deltaT = deltaT;

        return *this;
    }

    int getDeltaT() const {
        return deltaT;
    }

    TComm& setTimeOut(double timeOut) {
        this->timeOut = timeOut;

        return *this;
    }

    double getTimeOut() const {
        return timeOut;
    }

    TComm& setTrys(int trys) {
        this->trys = trys;

        return *this;
    }

    int getTrys() const {
        return trys;
    }

    TComm& setSingleShot(bool singleShot=true) {
        this->singleShot = singleShot;
        
        return *this;
    }

    bool isSingleShot() const {
        return singleShot;
    }
};



#endif	/* TCOMM_H */

