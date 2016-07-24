#ifndef COMMCTRL_H
#define COMMCTRL_H

#include "sysHeaders.h"
#include "stdHeaders.h"

int commWrite(string data, string port);

int commRead(string& data, string port);

#endif // COMMCTRL_H
