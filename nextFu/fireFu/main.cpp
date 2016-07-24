#include "commCtrl.h"

int main(int argc, char *argv[])
{
    string data;
    while (1) {

        string sendData;

        //cin >> sendData;

        //commWrite(sendData, "/dev/ttyUSB0");

        commRead(data, "/dev/ttyUSB0");

        cout << data << endl;
    }

    return 0;
}
