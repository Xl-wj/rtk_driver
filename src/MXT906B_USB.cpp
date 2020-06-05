#include "MXT906B_USB.h"

#include<termios.h>
#include<unistd.h>
#include<stdlib.h>
#include<fcntl.h>
#include<linux/kernel.h>
#include<stdio.h>

#include<string.h>
#include<sys/select.h>
#include<sys/time.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<netinet/in.h>
#include<errno.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;

#define PORT  6666
#define DeviceName "/dev/ttyUSB0"
#define DeviceSpeedDef  B115200
#define ParityBitDef
#define DataBitDef           8
#define StopBitDef           1

char buf[256];
int ComFd;
const int MAXBUFPIECE = 80;
vector<string> stringSplit(const string& s, const string& delimiter) {
  size_t pos1 = 0;
  size_t pos2 = 0;

  vector<string> result;
  while ((pos2 = s.find(delimiter, pos1)) != string::npos) {
    if (pos1 != pos2) result.emplace_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + delimiter.length();
  }
  if (pos1 != s.size()) result.push_back(s.substr(pos1));
  return result;
}

bool RosGPSMsge(const NMEA0183 &data_parse, sensor_msgs::NavSatFix &gps_msgs) {
	auto buf = data_parse.GetSentence();

        string sentence;
        for(int i = 0; i < MAXBUFPIECE; i++) sentence.push_back(buf[i]);

cout << "s = " << sentence << endl;
        for(int i=0; i<sentence.size()-1; i++) {
           if(sentence[i] == ',' && sentence[i+1] == ',') return false;       
        }
        cout << "sentence: " << sentence << endl;
        auto strs = stringSplit(sentence, ",");
        cout << "strs. size = " << strs.size() << endl;
        // is not euqal to GPSPrefix
        if(strs.size() < 10 && strs[0] != GPSPrefix) return false; 
        gps_msgs.header.stamp = ros::Time(stod(strs[1]));
	//gps_msgs.status.status = data_parse.GetState();
        
        if(strs[2] == " " || strs[4] == " " || strs[9] == " ") return false;

	gps_msgs.latitude = stod(strs[2]) / 100.0;
        if(gps_msgs.latitude < 0 || gps_msgs.latitude > 90.0) return false;

        gps_msgs.longitude = stod(strs[4]) / 100.0;
        if(gps_msgs.longitude < -180 || gps_msgs.longitude > 180.0) return false;

  	gps_msgs.altitude = stod(strs[9]); 
        gps_msgs.status.status = stoi(strs[6]);

        gps_msgs.header.stamp = ros::Time::now();
        cout << "stamp: " << gps_msgs.header.stamp << endl;

        return true;
}

void MXT906BUSB::SetCom(int fd,int BaudRate,int DateBit,int StopBit){
    struct  termios    CmdCfg1,Cmdfg2;
    tcgetattr(fd,&Cmdfg2);
    CmdCfg1=Cmdfg2;
    cfmakeraw(&CmdCfg1);
    CmdCfg1.c_cflag&=~CSIZE;
    cfsetispeed(&CmdCfg1,BaudRate);
    cfsetospeed(&CmdCfg1,BaudRate); //
    CmdCfg1.c_cflag&=~CSIZE;
    CmdCfg1.c_cflag|=CS8;
    CmdCfg1.c_cflag&=~PARENB;
    CmdCfg1.c_iflag&=~INPCK;

    CmdCfg1.c_cflag&=~CSTOPB;
    CmdCfg1.c_cc[VTIME] = 1;
    CmdCfg1.c_cc[VMIN] = 1;
    tcflush(fd,TCIFLUSH);
    tcsetattr(fd,TCSANOW,&CmdCfg1);
}

int MXT906BUSB::ComRecv(int fd,char *buf,int datelen){
    int len,fs_sel;
    fd_set  fs_read;
    struct timeval time;
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    time.tv_sec=5;
    time.tv_usec=0;

    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel) {
        len = read(fd,buf,datelen);
//        printf("len =%d,fs_sel=%d\n",len,fs_sel);
        return len;
    }
    else
        return 0;
}

void MXT906BUSB::ResetUSB() {
    ComFd = open(DeviceName,O_RDWR);

    if(ComFd ==-1) {
        cout << "Open Com failed; error: " << strerror(errno) << endl;
        return ;
    }

    SetCom(ComFd,DeviceSpeedDef,DataBitDef,StopBitDef);
}



static std::string sentence;


void MXT906BUSB::run() {
  cout << "MXT906BUSB::run!" << endl;
  while(true) {
    int r_count = ComRecv(ComFd, buf, sizeof(buf));

    if(r_count) {
            for(int i = 0; i < r_count; i++)
                sentence.push_back(buf[i]);

            auto strs = stringSplit(sentence, "$");

            for(int i=0; i<strs.size()-1; i++) {
                string str = strs[i];

                if(str.size() < 30) continue;
                if(str.substr(0, 5) == "GNGGA") {
                    buffer.push(str);
                }
            }
            sentence = strs.back();
    }
  }

}



void MXT906BUSB::publish() {
  std::cout << " MXT906BUSB::publish! " << std::endl;
  ros::NodeHandle nn;
  ros::Publisher gps_pub = nn.advertise<sensor_msgs::NavSatFix>("gps_pub", 50);
  ros::Rate loop_rate(20);


  NMEA0183 data_parse;

  while(ros::ok()) {

      if(!buffer.empty()) {
           string gps_msgs = "$" + buffer.front();
           buffer.pop();
   cout << "gps_msgs: " << gps_msgs << endl;
           for(int i=0; i<gps_msgs.size(); i++) {
              if (data_parse.Update(gps_msgs[i])) {
                    cout << "update" << endl;
//ros::Time begin = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        cout << "current_time: " << current_time << endl;
		      sensor_msgs::NavSatFix gps_msgs;
		      if(RosGPSMsge(data_parse, gps_msgs)) {
cout << "status = " << int(gps_msgs.status.status) << endl;
                            gps_pub.publish(gps_msgs);
		      }
	      }
           }

      }
      loop_rate.sleep();
  }
}


