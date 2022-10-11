#ifndef MYSOCKET_H
#define MYSOCKET_H

#include <iostream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>

using namespace std;

class Socket{
    private:
        int sock;
        int bind_ok;
        struct sockaddr_in from;
        struct sockaddr_in to;
        int to_size;
    
    public:
        Socket(bool is_first_received, int port, const char* to_addr);
        ~Socket();
        int get_sock();
        struct sockaddr_in* get_from_ptr();
        struct sockaddr_in* get_to_ptr();
        int* get_to_size_ptr();


        bool socketOK();
        bool bindOK();
        void setTimeout(int ms);

};

Socket::Socket(bool is_first_received, int port, const char* to_addr = NULL){
    memset(&from, 0, sizeof(from));
    memset(&to, 0, sizeof(to));
    sock = socket(PF_INET, SOCK_DGRAM, 0);
    to_size = sizeof(to);
    from.sin_family = AF_INET;
    from.sin_port = htons(port);
    from.sin_addr.s_addr = htonl(INADDR_ANY);
    if(is_first_received){
        bind_ok = bind(sock, (struct sockaddr *)&from, sizeof(from));
    }else{
        to.sin_family = AF_INET;
        to.sin_port = htons(port);
        to.sin_addr.s_addr = inet_addr(to_addr);
        bind_ok = 1;
    }

}

Socket::~Socket(){
    close(sock);
}

int Socket::get_sock(){
    return sock;
}

struct sockaddr_in* Socket::get_to_ptr(){
    return &to;
}
int* Socket::get_to_size_ptr(){
    return &to_size;
}

bool Socket::socketOK(){
    if(sock == -1){
        printf("socket generating fail");
        return false;
    }
    return true;
}

bool Socket::bindOK(){
    if(bind_ok == -1){
        printf("bind error");
        return false;
    }
    return true;
}

void Socket::setTimeout(int ms){
    struct timeval optVal = {0, ms};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &optVal, sizeof(optVal));
}

#endif