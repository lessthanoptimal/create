#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>

void myWrite( int fd , uint8_t *data, int size ) {
 if (write(fd, data, size) < size) {
   perror("write()");
   exit(0);
 }
}

void sendStart( int fd ) {
 uint8_t data[1] = {128};
 myWrite(fd,data,1);
}

void sendFull( int fd ) {
 uint8_t data[1] = {132};
 myWrite(fd,data,1);
}

void sendDemo( int fd , int which ) {
 uint8_t data[2] = {136,which};
 myWrite(fd,data,2);
}

void setDigitalIO( int fd , uint8_t value) {
 uint8_t data[2] = {147,value};
 myWrite(fd,data,2);
}

int main (int argc, char *argv[])
{
 if (argc < 2) {
   fprintf(stdout,"Usage: %s [port]\n",argv[0]);
   return 1;
 }

 // open the serial port
 int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
 if( fd == -1 ) {
   perror("open()");
   exit(0);
 }

 // configure the serial port
 struct termios options;
 tcgetattr(fd, &options);

 if( cfsetispeed(&options, B57600) == -1 )  {
   perror("cfsetispeed()");
   exit(0);
 }
 if( cfsetospeed(&options, B57600) == -1 ) {
   perror("cfsetispeed()");
   exit(0);
 }

 // 8N1 no flow control
 options.c_cflag &= ~PARENB;
 options.c_cflag &= ~CSTOPB;
 options.c_cflag &= ~CSIZE;
 options.c_cflag &= ~CRTSCTS;
 options.c_cflag |= CS8;

 if( tcsetattr(fd, TCSANOW, &options) == -1 ) {
   perror("tcsetattr()");
   exit(0);
 }

 sendStart(fd);
 sendFull(fd);
 sendDemo(fd,0);
// setDigitalIO(fd,0xFF);

 return 0;
}
