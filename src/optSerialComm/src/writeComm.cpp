#include "ros/ros.h"
#include "../../include/stdHeader.h"
#include "../../include/commHeader.h"

///<可以修改的区域>
#define BUFSIZE 512
#define BUAD B9600  //注意前面有个B
///</可以修改的区域>
#define TRUE 1

using namespace std;

ros::Publisher data_pub;
int myData[5], lastData[5];
std_msgs::String pubData;

int fd;
int nCount, i;
struct termios oldtio, newtio;
char dev[13] = "/dev/ttyACM0";
char cmd_data[BUFSIZE] = {'\0'};
int isFirst = 1;

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
///</不可以被修改的区域>

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{

    string cmd_data = (char*)msgs->data.c_str();
    cmd_data += '\0';
    ROS_INFO("I get the data: [%s] \r\n", cmd_data.c_str());

    if(isFirst)
    {
        if ((fd = open(dev, O_RDWR | O_NOCTTY))<0)
        {
        // 错误1：检测不到端口，需要重新检查端口并设置
           printf("err: can't open serial port!\n");
           return;
        }
    
        // Set the serial commutation
        tcgetattr(fd, &oldtio); /* save current serial port settings */
        setTermios(&newtio, BUAD);
    
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &newtio);
        isFirst = 0;
    }
    nCount=write(fd, cmd_data.c_str(), cmd_data.length());

    usleep(50000);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "SerialWrite");
    ros::NodeHandle n;

    // if ((fd = open(dev, O_RDWR | O_NOCTTY))<0)
    // {
    // // 错误1：检测不到端口，需要重新检查端口并设置
    //    printf("err: can't open serial port!\n");
    //    return -1;
    // }
 
    // // Set the serial commutation
    // tcgetattr(fd, &oldtio); /* save current serial port settings */
    // setTermios(&newtio, BUAD);
 
    // tcflush(fd, TCIFLUSH);
    // tcsetattr(fd, TCSANOW, &newtio);

    // char* test ="111111111111";
    // nCount=write(fd, test, strlen(test));

    // Binding the da
    data_pub = n.advertise<std_msgs::String>("SerialWrite", 1000);

    ros::Subscriber sub = n.subscribe("CvtCommand", 1000, chatterCallback);

    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);

    ros::spin();

    return 0;
}


//==============================================

// the content of the table

// +--------+----------+----------------+----------+
// | itemID | itemName | objectCategory | position |
// +--------+----------+----------------+----------+ 
