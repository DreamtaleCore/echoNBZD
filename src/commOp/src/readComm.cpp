#include "ros/ros.h"
#include "../../include/stdHeader.h"
#include "../../include/commHeader.h"
#include <unistd.h>
#include <string>
#define TRUE 1
#define BUFSIZE 512
#define PORT ("/dev/ttyUSB0")

ros::Publisher data_pub;
int myData[5], lastData[5];
std_msgs::String pubData;

//初始化串口选项：  
void setTermios(struct termios * pNewtio, int uBaudRate)
{
    bzero(pNewtio, sizeof(struct termios)); /* clear struct for new port settings */

    //8N1
    pNewtio->c_cflag = uBaudRate | CS8 | CREAD | CLOCAL;
    pNewtio->c_iflag = IGNPAR;

    pNewtio->c_oflag = 0;
    pNewtio->c_lflag = 0; //non ICANON
    /*
     initialize all control characters
     default values can be found in /usr/include/termios.h, and
     are given in the comments, but we don't need them here
     */
    pNewtio->c_cc[VINTR] = 0; /* Ctrl-c */
    pNewtio->c_cc[VQUIT] = 0; /* Ctrl-\ */
    pNewtio->c_cc[VERASE] = 0; /* del */
    pNewtio->c_cc[VKILL] = 0; /* @ */
    pNewtio->c_cc[VEOF] = 4; /* Ctrl-d */
    pNewtio->c_cc[VTIME] = 5; /* inter-character timer, timeout VTIME*0.1 */
    pNewtio->c_cc[VMIN] = 0; /* blocking read until VMIN character arrives */
    pNewtio->c_cc[VSWTC] = 0; /* '\0' */
    pNewtio->c_cc[VSTART] = 0; /* Ctrl-q */
    pNewtio->c_cc[VSTOP] = 0; /* Ctrl-s */
    pNewtio->c_cc[VSUSP] = 0; /* Ctrl-z */
    pNewtio->c_cc[VEOL] = 0; /* '\0' */
    pNewtio->c_cc[VREPRINT] = 0; /* Ctrl-r */
    pNewtio->c_cc[VDISCARD] = 0; /* Ctrl-u */
    pNewtio->c_cc[VWERASE] = 0; /* Ctrl-w */
    pNewtio->c_cc[VLNEXT] = 0; /* Ctrl-v */
    pNewtio->c_cc[VEOL2] = 0; /* '\0' */
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "readComm");
    ros::NodeHandle nh;
    data_pub = nh.advertise<std_msgs::String >("readComm", 1000);

    // int fd;
    // int nread;
    // char buff[BUFSIZE];
    // struct termios oldtio, newtio;
    // struct timeval tv;
    // fd_set rfds;

    // if ((fd = open(PORT, O_RDWR | O_NOCTTY))<0)
    // {
    //     printf("err: can't open serial port!\n");
    //     return -1;
    // }

    // tcgetattr(fd, &oldtio); /* save current serial port settings */
    // setTermios(&newtio, B9600);

    // tcflush(fd, TCIFLUSH);
    // tcsetattr(fd, TCSANOW, &newtio);

    // tv.tv_sec=30;
    // tv.tv_usec=0;
    // char print_line[BUFSIZE];
    // int pointer = 0;
    // FD_ZERO(&rfds);
    // FD_SET(fd, &rfds);
    while (TRUE)
    {
/*        if (select(1+fd, &rfds, NULL, NULL, &tv)>0)
        {
            if (FD_ISSET(fd, &rfds))
            {
                nread=read(fd, buff, BUFSIZE);
                //   printf("readlength = %d\n", nread);
                buff[nread]='\0';
                int i = 0;
                for(i = 0; buff[i] != '\0'; i++)
                {
                    print_line[pointer++] = buff[i];
                    if(buff[i]=='\n')
                    {
                        print_line[pointer] = '\0';*/
                        //print_line  = "hello world !";
                        char* aaaa = "05000hello\n";
                        ///aaaa[1] += 5;

                        //string aaaa = "hello world\n";
                        //printf("%s", print_line);
                        printf("%s", aaaa);
                        // Pubish the data from comm port
                        //pubData.data = print_line;
                        pubData.data = aaaa;
                        data_pub.publish(pubData);
                        sleep(1);
/*                        pointer = 0;
                    }
                }
            }
        }*/
    }
   /* tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);*/
    return 0;
}
