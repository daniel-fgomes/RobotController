/* 
 * File:   SerialPort.h
 * Author: daniel
 *
 * Created on April 15, 2014, 11:54 PM
 */

#ifndef SERIALPORT_H
#define	SERIALPORT_H

#include <iostream>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/io.h>
#include <string>
#include <assert.h>

//Ver ajuda em: http://en.wikibooks.org/wiki/Serial_Programming/termios
#define DEFAULTMODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */

//int wait_flag = true; //TRUE while no signal received

//void signal_handler_IO(int status)//definition of signal handler
//{
//    printf("received SIGIO signal.\n");
//    wait_flag = false;
//}

class SerialPort {
private:
    int fd, res;
    struct termios oldtio, newtio;
    //    struct sigaction saio; /* definition of signal action */
    char buf[255];
    bool asyncmode;
public:

    SerialPort() {
        fd = 0;
        asyncmode = false;
    }

    ~SerialPort() {
        closePort();
    }
    
SerialPort& set(int baudrate = B115200) {
        int rc;

        if ((rc = tcgetattr(fd, &oldtio)) < 0) {
            fprintf(stderr, "failed to get attr: %d, %s\n", fd, strerror(errno));
            assert(0);
        }

        memset(&newtio, 0, sizeof (newtio)); /* clear the new struct */

        newtio.c_cflag = CS8 | IGNPAR | 0 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        newtio.c_lflag = 0;
        newtio.c_cc[VMIN] = 0; /* block untill n bytes are received */
        newtio.c_cc[VTIME] = 0; /* block untill a timer expires (n * 100 mSec.) */



        cfsetispeed(&newtio, baudrate);
        cfsetospeed(&newtio, baudrate);

        if ((rc = tcsetattr(fd, TCSANOW, &newtio)) < 0) {
            fprintf(stderr, "failed to set attr: %d, %s\n", fd, strerror(errno));
            assert(0);
        }

        clearBuffer();

        return *this;
    }

    bool openPort(std::string port = "") {

        if (port == "")
            port = DEFAULTMODEMDEVICE;

        /* open the device to be non-blocking (read will return immediatly) */
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (fd == -1) {
            fprintf(stderr, "openPort: Unable to open %s %s\n", port.c_str(), strerror(errno));
            assert(0);
        }

        return true;
    }

    bool isOpen() {
        if (fd > 0) return true;

        return false;
    }

    SerialPort& raw(bool v, bool asyncmodeIn = false) {

        return *this;
    }

    void closePort() {
        if (fd != 0) {
            tcsetattr(fd, TCSANOW, &oldtio);
            close(fd);
        }
    };

    int readPort(unsigned char *buf, const int Nmax, int &res) {

        res = read(fd, buf, Nmax);

        return res;
    }

    int writePort(unsigned char *buf, const int N) {
        
        return write(fd, buf, N);
    }
    
    SerialPort& clearBuffer()
    {
        tcflush(fd,TCIOFLUSH);
        return *this;
    }
};

#endif	/* SERIALPORT_H */

