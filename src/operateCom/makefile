CC = /usr/local/arm/2.95.3/bin/arm-linux-gcc
all:receive send
receive: receive.c
    $(CC) receive.c -o  receive
send: send.c
    $(CC) send.c -o  send
clean:
    -rm -rf testCOM receive send
