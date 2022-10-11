#include "../include/udp_base/packet.h"
#include "udp_base/wplist.h"
#include "std_msgs/Int64.h"
#include <vector>
#define RATE 10

using namespace std;

Packet wpl_pk(WPL);
Packet cmd_pk(CMD);
Socket sk(true, 21000);

void wplist_Callback(const udp_base::wplist::ConstPtr& msg){
    static int id = -1;
    if(id != msg->id){
        Payload *wpl_pl = new Payload;
        wpl_pl->wpl.way_count = msg->size;
        wpl_pl->wpl.id = msg->id;
        for(int i = 0; i<msg->size; i++){
            wpl_pl->wpl.wplist[i][0] = msg->x[i];
            wpl_pl->wpl.wplist[i][1] = msg->y[i];
            wpl_pl->wpl.wplist[i][2] = msg->z[i];
            wpl_pl->wpl.wplist[i][3] = msg->s[i];
        }

        wpl_pk.set_payload(wpl_pl);
        wpl_pk.sendTo(0);
        wpl_pk.view_packet(S);
        wpl_pk.inc_pnum();
        delete wpl_pl;
        id = msg->id;
    }
}

void cmd_Callback(const std_msgs::Int64::ConstPtr& msg){
    static int val = -1;
    if(val != msg->data){
        Payload *cmd_pl = new Payload;
        cmd_pl->cmd.cmd = msg->data;
        cmd_pk.set_payload(cmd_pl);
        cmd_pk.sendTo(0);
        cmd_pk.view_packet(S);
        cmd_pk.inc_pnum();
        delete cmd_pl;
        val = msg->data;
    }
}

int main(int argc, char **argv)
{
    if(!sk.socketOK() || !sk.bindOK()){
        exit(1);
    }
    Packet *recv_pk = new Packet();
    recv_pk->set_socket(&sk);
    wpl_pk.set_socket(&sk);
    cmd_pk.set_socket(&sk);

    ros::init(argc, argv, "udp_base");
    ros::NodeHandle nh;
    ros::Rate loop_rate(RATE);

    ros::Subscriber wpl_sub = nh.subscribe<udp_base::wplist>("/wplist", 100, wplist_Callback);

    ros::Subscriber cmd_sub = nh.subscribe<std_msgs::Int64>("/cmd_base", 100, cmd_Callback);

    while (ros::ok())
    {
        if (recv_pk->recvFrom(0) >= 0)
        {
            recv_pk->view_packet(R);
        }
        else
        {
            ROS_ERROR("nothing received");
            cout<<"======================================================"<< endl;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    delete recv_pk;
    return 0;
}
