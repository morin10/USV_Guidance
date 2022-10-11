#ifndef PACKET_H
#define PACKET_H

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "cmath"
#include <sys/types.h>
#include "payload.h"
#include "mysocket.h"

#define BUFSIZE 8192

typedef enum pk_type
{
    ANON = -1,
    DEV = 0,
    RSP = 1,
    WPL = 2,
    CMD = 3,
}pk_type;

typedef enum sr_type
{
    S = 0,
    R = 1,
}sr_type;

struct packet
{
    pk_type type;  // Packet ID
    int PacketNum; // Packet sequential number
    Payload payload;
};

class Packet
{
private:
    struct packet *packet;
    Socket *socket;

public:
    Packet(pk_type type);
    ~Packet();
    void set_packet(char *buf);
    void set_type(pk_type type);
    void set_payload(Payload *_payload);
    void inc_pnum();
    pk_type get_type();
    int get_pnum();
    Payload* get_payload();
    void view_packet(sr_type type);

    void set_socket(Socket *_socket);
    int sendTo(int flags);
    int recvFrom(int flags);
};

Packet::Packet(pk_type type = ANON)
{
    packet = new struct packet;
    memset(packet, 0, sizeof(*packet));
    packet->type = type;
}


Packet::~Packet()
{
    if(packet != NULL){
        delete packet;
    }
}

void Packet::set_packet(char *buf)
{
    memcpy(packet, &buf, sizeof(packet));
}

void Packet::set_type(pk_type type)
{
    packet->type = type;
}

void Packet::set_payload(Payload *_payload)
{
    memcpy(&packet->payload, _payload, sizeof(*_payload));
}

void Packet::inc_pnum(){
    packet->PacketNum++;
}

pk_type Packet::get_type()
{
    return packet->type;
}
int Packet::get_pnum()
{
    return packet->PacketNum;
}

Payload* Packet::get_payload(){
    return &packet->payload;
}

void Packet::view_packet(sr_type type)
{   
    if(type){
        ROS_INFO("RECEIVE PACKET");
    }else{
        ROS_INFO("SEND PACKET");
    }
    std::cout<<"------------------------------------------------------"<< std::endl;

    switch (packet->type)
    {
    case DEV:
        ROS_INFO("PACKET TYPE : DEV");
        ROS_INFO("PACKET NUMBER : %d", packet->PacketNum);
        ROS_INFO("DEVICE NAME : %s", packet->payload.dev.dev_name);
        switch (packet->payload.dev.dev_type)
        {
        case 1:
            ROS_INFO("DEVICE TYPE : USV");
            break;
        case 2:
            ROS_INFO("DEVICE TYPE : UAV");
            break;
        default:
            ROS_ERROR("DEVICE TYPE : UNKNOWN(%d)", packet->payload.dev.dev_type);
            break;
        }
        ROS_INFO("DEVICE ID : %d", packet->payload.dev.dev_ID);
        ROS_INFO("DEVICE SEARCH RANGE : %d", packet->payload.dev.search_range);
        ROS_INFO("UTM X : %.3f", packet->payload.dev.utm_x);
        ROS_INFO("UTM Y : %.3f", packet->payload.dev.utm_y);
        ROS_INFO("ALTITUDE : %.1f", packet->payload.dev.altitude);
        ROS_INFO("COG : %.1f", packet->payload.dev.cog);
        ROS_INFO("SOG : %.1f", packet->payload.dev.sog);
        ROS_INFO("MODE : %d", packet->payload.dev.mode);
        break;

    case RSP:
        ROS_INFO("PACKET TYPE : RSP");
        ROS_INFO("PACKET NUMBER : %d", packet->PacketNum);
        switch (packet->payload.rsp.ptype)
        {
        case WPL:
            ROS_INFO("RESPONED PACKET TYPE : WPL");
            break;
        case CMD:
            ROS_INFO("RESPONED PACKET TYPE : CMD");
            break;
        default:
            ROS_INFO("RESPONED PACKET TYPE : UNKNOWN(%d)", packet->payload.rsp.ptype);
            break;
        }
        ROS_INFO("RESPONED PACKET NUMBER : %d", packet->payload.rsp.pnum);
        break;

    case WPL:
        ROS_INFO("PACKET TYPE : WPL");
        ROS_INFO("PACKET NUMBER : %d", packet->PacketNum);
        ROS_INFO("NUM OF WAY POINT : %d", packet->payload.wpl.way_count);
        for (int i = 0; i < packet->payload.wpl.way_count; i++)
        {
            ROS_INFO("WAY POINT%d : x[%.3f], y[%.3f], z[%.3f], s[%d]", i,
                     packet->payload.wpl.wplist[i][0],
                     packet->payload.wpl.wplist[i][1],
                     packet->payload.wpl.wplist[i][2],
                     (int)packet->payload.wpl.wplist[i][3]);
        }
        break;

    case CMD:
        ROS_INFO("PACKET TYPE : CMD");
        ROS_INFO("PACKET NUMBER : %d", packet->PacketNum);
        ROS_INFO("COMMAND : %d", packet->payload.cmd.cmd);
        break;
    default:
        ROS_ERROR("INVALID TYPE : %d", packet->type);
        break;
    }

    std::cout<<"======================================================"<< std::endl;
}

void Packet::set_socket(Socket *_socket){
    socket = _socket;
}

int Packet::sendTo(int flags)
{
    int sock = socket->get_sock();
    sockaddr_in* to = socket->get_to_ptr();
    return sendto(sock, (char *)packet, sizeof(*packet), flags, (sockaddr *)to, sizeof(*(sockaddr *)to));
}

int Packet::recvFrom(int flags)
{
    int sock = socket->get_sock();
    sockaddr_in* to = socket->get_to_ptr();
    int* to_size = socket->get_to_size_ptr();
    return recvfrom(sock, (char *)packet, sizeof(*packet), flags, (sockaddr *)to, (socklen_t *)to_size);
}

#endif
