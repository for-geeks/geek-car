#ifndef LISENER_H
#define LISENER_H
#include <stdio.h>
#include <stdlib.h>
#include <sys/msg.h>
int start_cyber(void);
class MessageCom{
public:
    MessageCom(void){};
    ~MessageCom(void){};
    void msg_init(void);
    int msqid;
private:
    key_t key;
    const char* MSG_FILE = "etc/passwd";
};
#endif
