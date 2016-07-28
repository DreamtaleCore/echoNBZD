#include "ros/ros.h"
#include "../../include/stdHeader.h"
#include "../../include/socketHeader.h"

#define MAXLINE 4096

char* ip = "10.50.142.206";
//char* ip = "10.50.142.1";
#define PORT 12345

ros::Publisher data_pub;
std_msgs::String pubData;

void chatterCallback(const std_msgs::String::ConstPtr& msgs)
{

    //int dataSum = str2data(msgs->data.c_str(), ',', myData);
    int    sockfd, n;
    char    recvline[4096], sendline[4096];
    struct sockaddr_in    servaddr;

    ROS_INFO("I detect the msg: [%s]\n", msgs->data.c_str());
    
    if( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);
        exit(0);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    if( inet_pton(AF_INET, ip, &servaddr.sin_addr) <= 0){
        printf("inet_pton error for %s\n",ip);
        return;
    }

    if( connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno);
        return;
    }

    printf("send msg to server: \n");
    strcpy(sendline, msgs->data.c_str());
    if( send(sockfd, sendline, strlen(sendline), 0) < 0)
    {
        printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
        return;
    }
    //recv(sockfd, sendline, strlen(sendline), 0);
    printf("%s\n", sendline);

    close(sockfd);
    
    return;    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mySocket");
    ros::NodeHandle n;

    data_pub = n.advertise<std_msgs::String>("mySocket", 1000);

    ros::Subscriber sub = n.subscribe("readComm", 1000, chatterCallback);

    ros::spin();

    return 0;
}