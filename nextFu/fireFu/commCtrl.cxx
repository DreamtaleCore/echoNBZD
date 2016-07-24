#include "commCtrl.h"

int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {  38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,
                    19200,  9600, 4800, 2400, 1200,  300, };

/**
 * @brief   Open the communication device
 * @param   the Serial port number
 * @return  the device id
 */
int openDev(char *Dev)
{
    int fd = open( Dev, O_RDWR | O_NOCTTY );  //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return -1;
    }
    else
        return fd;
}

/**
 * @brief set the communicate speed
 * @param the device number
 * @param speed of comm
 */
void setSpeed(int fd, int speed)
{
    int   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);
            if  (status != 0)
            {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

/**
 * @brief setParity
 * @param fd
 * @param databits
 * @param stopbits
 * @param parity
 * @return
 */
int setParity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    options.c_oflag  &= ~OPOST;   /*Output*/
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*set the data bits*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size/n"); return (FALSE);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;    /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* set as odd parity*/
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;     /* Enable parity */
            options.c_cflag &= ~PARODD;    /* cvt even parity*/
            options.c_iflag |= INPCK;      /* Disnable parity checking */
            break;
        case 'S':
        case 's':                           /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity/n");
            return (FALSE);
    }
    /* set the stop bit*/
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
           break;
        default:
             fprintf(stderr,"Unsupported stop bits/n");
             return (FALSE);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150;      /* set timeup 15 seconds*/
    options.c_cc[VMIN] = 0;         /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

/**
 * @brief commRead
 * @param data
 * @param port
 * @return
 */
int commRead(string& data, string port)
{
    int fd;
    int nread;
    char buff[512];
    fd = openDev((char*)port.c_str());
    setSpeed(fd, 9600);
    if (setParity(fd, 8, 1,'N') == FALSE)
    {
        printf("Set Parity Error/n");
        return -1;
    }
    while((nread = read(fd, buff, 512))>0)
    {
        buff[nread+1] = '\0';
        if(strlen(buff) > 1)
            break;
    }
    data = buff;
    return 0;
}

/**
 * @brief commWrite
 * @param data
 * @param port
 * @return
 */
int commWrite(string data, string port)
{
    int fd;
    int nwrite;
    fd = openDev((char*)port.c_str());
    setSpeed(fd, 9600);
    if (setParity(fd,8,1,'N') == FALSE)
    {
        printf("Set Parity Error/n");
        return -1;
    }
    nwrite = write(fd, data.c_str() ,data.length());
    if(nwrite == -1)
    {
        printf("Wirte sbuf error./n");
        return -1;
    }
   
}
