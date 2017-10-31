#ifndef _SELIDAR_DRIVER_H_
#define _SELIDAR_DRIVER_H_

#include <Thread/Condition.h>
#include "SelidarTypes.h"
#include "Serial.h"
#include "SelidarProtocol.h"

namespace NS_Selidar
{
  
#define DEFAULT_TIMEOUT 2000
#define MAX_SCAN_NODES 2048
  
#define _M_PI  (3.1415927)
#define I_RAD ((_M_PI)/180.0)
#define DEG2RAD(x) ((x)*_M_PI/180)

  struct LaserDataNode{
	  unsigned short distance;
	  unsigned short angle;
  };
  struct LidarConstant
  {
	double angle_min;
	double angle_max;
	double angle_increment;
	double range_min;
	double range_max;
	double kDistance;
  };

  class SelidarDriver
  {
  private:
	unsigned char *recv_buf;
	unsigned char *buf_head;
	unsigned char *buf_rear;
	unsigned char crc(unsigned char *buf);

  public:
    SelidarDriver ();
    virtual
    ~SelidarDriver ();
    LidarConstant ldConst;

  public:
    bool init(int align_cnt);
    bool
    decode(LaserDataNode *node, unsigned char *buf);
    int
    connect (const char * port_path, unsigned int baudrate, unsigned int flag);
    void
    disconnect ();
    bool
    isConnected ();
    int
    stop ();

    int
    startScan ();
    int
	getOnePoint(LaserDataNode *ldn);

    bool connected;
    bool scanning;

    boost::mutex rxtx_lock;
    NS_NaviCommon::Condition data_cond;
    Serial* rxtx;
  };

}

#endif
