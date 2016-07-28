///< 不可以被修改的区域 >
#include   <stdio.h>     
#include   <stdlib.h>   
#include   <unistd.h>     
#include   <sys/types.h> 
#include   <sys/stat.h>  
#include   <fcntl.h>    
#include   <termios.h>  
#include   <errno.h>    
#include   <string.h>

#define TRUE 1
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
///<可以修改的区域>
#define BUFSIZE 512
#define PORT "/dev/ttyACM0"
#define BUAD B9600  //注意前面有个B
///</可以修改的区域>
int main(int argc, char **argv)
{
    int fd;
    int nCount, i;
    struct termios oldtio, newtio;
    char *dev = PORT;
	char cmd_data[BUFSIZE] = {'\0'};
 
    if ((fd = open(dev, O_RDWR | O_NOCTTY))<0)
    {
    // 错误1：检测不到端口，需要重新检查端口并设置
       printf("err: can't open serial port!\n");
       return -1;
    }
 
    tcgetattr(fd, &oldtio); /* save current serial port settings */
    setTermios(&newtio, BUAD);
 
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);
 	while(1)
	{
		printf("Input command data here:\n");
		scanf("%s", cmd_data);
 	
   		nCount=write(fd, cmd_data, strlen(cmd_data));
   		printf("send data\n");
   		//sleep(1);
		
		printf("Continue?(Y/N)");
		char c;
		scanf("\n%c", &c);
		if(c != 'y' && c != 'Y')
			break;
	}
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
}
