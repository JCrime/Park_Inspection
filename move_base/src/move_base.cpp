#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <cmath>
#include <move_base/move_base.h>

using namespace std;

//创建一个serial类
serial::Serial sp;
unsigned char cmd[11]={0x1a,0x2b,0x00,0x00,0x00,0x3c,0x00,0x00,0x00,0x01,0x5f};

void move_cb(const move_base::move_base::ConstPtr& msg)
{
    if(msg->header.frame_id == "base_link" && msg->isEmerge == false)
    {
        unsigned int vel = abs(msg->velocity);
        unsigned int ang = abs((int)(msg->angle*10));
        cmd[3]=(unsigned char)((vel>>8)&0xff);
        cmd[4]=(unsigned char)(vel&0xff);
        cmd[7]=(unsigned char)((ang>>8)&0xff);
        cmd[8]=(unsigned char)(ang&0xff);

        if(msg->velocity<0)
        {
            cmd[2]=0x00;
        }
        else cmd[2]=0x11;

        if(msg->angle<0)
        {
            cmd[6]=0x00;
        }
        else cmd[6]=0x11;

        sp.write(cmd,11);
        return;

    }
    else
    {
      //cout<<"Emerge received or wrong frame_id"<<endl;
      cmd[3]=cmd[4]=0x00;
      sp.write(cmd,11);
      return;
    }
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("move_base", 1000, move_cb);

    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyS1");
    //设置串口通信的波特率
    sp.setBaudrate(57600);
    //串口设置timeout
    sp.setTimeout(to);
    
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO("/dev/ttyS1 is opened.");
    }
    else
    {
        return -1;
    }

    ros::spin();

    return 0;
}
