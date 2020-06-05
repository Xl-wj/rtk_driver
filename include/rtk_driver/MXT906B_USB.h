#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <iostream>


#include "NMEA0183.h"
#include "sensor_msgs/NavSatFix.h"

const static std::string GPSPrefix = "$GNGGA";

std::vector<std::string> stringSplit(const std::string& s, const std::string& delimiter);

// for example:
// $GNGGA,093549.000,4006.718191,N,11618.399270,E,4,32,0.49,31.586,M,0,M,10,1286*6E
// $GNGGA,093551.000,4006.718188,N,11618.399270,E,4,32,0.49,31.596,M,0,M,12,1286*6C
bool RosGPSMsge(const NMEA0183 &data_parse, sensor_msgs::NavSatFix &gps_msgs);

std::queue<std::string> buffer;

class MXT906BUSB {
public:
    MXT906BUSB() {
        ResetUSB();
    }
    void ResetUSB();

    void run();

    void publish();

    void exec() {
        
        std::thread get_gps = std::thread(&MXT906BUSB::run, this);
        std::thread pub_gps = std::thread(&MXT906BUSB::publish, this);
        get_gps.join();
        pub_gps.join();
    }

    void SetCom(int fd,int BaudRate,int DateBit,int StopBit);
    int ComRecv(int fd,char *buf,int datelen);

};
