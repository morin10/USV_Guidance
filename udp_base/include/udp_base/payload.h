#ifndef PAYLOAD_H
#define PAYLOAD_H


typedef struct dev_payload
{
    char dev_name[20];
    int dev_type;
    int search_range;
    int dev_ID;

    double utm_x;
    double utm_y;
    double altitude;
    double cog;
    double sog;

    double length;
    double breadth;

    int mode;
}dev_payload;

typedef struct rsp_payload
{
    int ptype;
    int pnum;
}rsp_payload;

typedef struct wpl_payload
{
    int way_count;
    int id;
    double wplist[80][4];
}wpl_payload;

typedef struct cmd_payload
{
    int cmd;
}cmd_payload;

typedef struct Payload
{
    union
    {
        dev_payload dev;
        rsp_payload rsp;
        wpl_payload wpl;
        cmd_payload cmd;
    };
}Payload;

#endif