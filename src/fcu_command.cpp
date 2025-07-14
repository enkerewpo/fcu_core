#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <std_msgs/Int16.h>

static char buf[16] = {0};
static std_msgs::Int16 cmd;

int main(int argc, char **argv) {

  ros::init(argc, argv, "fcu_command");
  ros::NodeHandle nh;
  ros::Publisher command = nh.advertise<std_msgs::Int16>("/fcu_bridge/command",100);

  while (ros::ok()) {
    // 获取从键盘输入的数据
    printf("Please input command:\n");
    ssize_t size = read(STDIN_FILENO, buf, sizeof(buf));
    if(size>0){
      if(size!=2){
        printf("Command error!\n");
        continue;
      }
    }else{
      printf("Command disabled!\n");
      ros::shutdown();
      return 0;
    }
    switch(buf[0]){
      case 'a':
        printf("Unlock\n");
        cmd.data=1;
        command.publish(cmd);
        break;
      case 'd':
        printf("Lock\n");
        cmd.data=2;
        command.publish(cmd);
        break;
      case 't':
        printf("Takeoff\n");
        cmd.data=3;
        command.publish(cmd);
        break;
      case 'l':
        printf("Land\n");
        cmd.data=4;
        command.publish(cmd);
        break;
      case 'r':
        printf("Run\n");
        cmd.data=5;
        command.publish(cmd);
        break;
      case 's':
        printf("Stop\n");
        cmd.data=6;
        command.publish(cmd);
        break;
      case '1':
        printf("Position 1\n");
        cmd.data=7;
        command.publish(cmd);
        break;
      case '2':
        printf("Position 2\n");
        cmd.data=8;
        command.publish(cmd);
        break;
      case '3':
        printf("Position 3\n");
        cmd.data=9;
        command.publish(cmd);
        break;
      case '4':
        printf("Position 4\n");
        cmd.data=10;
        command.publish(cmd);
        break;
      default:
        printf("Invalid command!\n");
        break;
    }
  }

  return 0;
}
