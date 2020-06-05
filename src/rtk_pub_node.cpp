#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"

#include <thread>
#include <string>
#include <mutex>

#include <iostream>
#include <fstream>
#include "MXT906B_USB.h"
#include "NMEA0183.h"

using namespace std;

//const static string GPSPrefix = "$GNGGA";
sensor_msgs::NavSatFix::Ptr gps_msgs_ptr = nullptr;
std::mutex m_gps_msgs;


/*
struct SerialGps {
  string prefix;
  double utc_time;
  double lat;
  double lng;
  double height;
  string is_sn; // north/south
  string is_ew; // east /west
  int8_t state;
};*/




void testTXT() {
    cout << "test TXT gps data !" << endl;

    string file = "/home/xl/NMEA/20200529.dynamic.txt";
    fstream fs(file, ios::in);

    if(!fs.is_open()) {
        cout << "Failed to open the file: " << file << endl;
        return;

    } else {
        NMEA0183 pgs_parse;
    	string content;

   	while(getline(fs, content)) {

            for(int i=0; i<content.size(); i++) {
            	if (pgs_parse.Update(content[i])) {
                	sensor_msgs::NavSatFix gps_msgs;
			if(RosGPSMsge(pgs_parse, gps_msgs)) {
			    m_gps_msgs.lock();
			    gps_msgs_ptr = boost::make_shared<sensor_msgs::NavSatFix>(gps_msgs);
			    m_gps_msgs.unlock();
	 		}
	       }
	   }
         
       }
    }
}

void publish_thread() {

  ros::NodeHandle nn;
  ros::Publisher gps_pub = nn.advertise<sensor_msgs::NavSatFix>("gps_pub", 50);
  ros::Rate loop_rate(20);

  while(ros::ok()) {
      
      if(gps_msgs_ptr) {

         m_gps_msgs.lock();
         gps_pub.publish(*gps_msgs_ptr);
         gps_msgs_ptr = nullptr;
         m_gps_msgs.unlock();
          
      }
      loop_rate.sleep();
  }
}

int main(int argc,char **argv) {
  ros::init(argc, argv, "gps_pub_node");

  MXT906BUSB serial_port;
  serial_port.exec();
  
  //std::thread pub_gps = std::thread(&publish_thread_runtime);
  //pub_gps.join();

  
  
  
  
  return 0;
}
