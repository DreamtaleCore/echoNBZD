#ifndef SYSHEADERS_H
#define SYSHEADERS_H

#include <stdio.h>
#include <stdlib.h>     /*standard lib*/
#include <unistd.h>     /*Unix standard func*/
#include <sys/types.h>
#include <sys/stat.h>
#include "string.h"
#include <fcntl.h>      /*file control lib*/
#include <termios.h>    /*PPSIX terminal*/
#include <errno.h>      /*error information*/

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#endif // SYSHEADERS_H
