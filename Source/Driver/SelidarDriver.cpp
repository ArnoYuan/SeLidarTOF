#include <stdio.h>
#include <iostream>
#include "SelidarDriver.h"
#include "SelidarTypes.h"
#include <Time/Utils.h>
#include <Console/Console.h>
#include <string.h>
#include <Time/Rate.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using std::cin;
using std::cout;
using std::endl;

namespace NS_Selidar
{
#define LIDAR_CMD_START	"#SF\r\n"
#define LIDAR_CMD_STOP		"#SF 0\r\n"

static unsigned char cbit[256]={
	    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,
	    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
	    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
	    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
	    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
	    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
	    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
	    4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8,
};


// Serial Driver Impl
  
  SelidarDriver::SelidarDriver ()
      : connected (false), scanning (false)
  {
    rxtx = new Serial ();
    recv_buf = new unsigned char [1024];
    memset((void *)recv_buf, 0,1024);
    buf_head = recv_buf;
    buf_rear = recv_buf;

  }
  
  SelidarDriver::~SelidarDriver ()
  {
    // force disconnection
    disconnect ();

    delete rxtx;
  }
  
  bool
  SelidarDriver::init(int align_cnt)
  {
	ldConst.angle_min = -_M_PI + I_RAD / 16;
	ldConst.angle_max = _M_PI;
	ldConst.angle_increment = _M_PI * 2 / align_cnt;
	ldConst.range_min = 0.0;
	ldConst.range_max = 40.0;
	ldConst.kDistance = 0.01;
  }

  unsigned char
  SelidarDriver::crc(unsigned char *buf)
  {
	  return (cbit[buf[1]]+cbit[buf[2]]+cbit[buf[3]])&0x07;
  }

  bool
  SelidarDriver::decode(LaserDataNode *node, unsigned char *buf)
  {
	unsigned char crcdata = crc(buf);
	unsigned char crcdata1 = (buf[0] >> 4) & 0x07;
	if (crcdata1 != crcdata)

		return false;

	unsigned short ustemp;

	//compute distance
	ustemp = (buf[0] & 0x0F);
	ustemp <<= 7;
	ustemp += (buf[1] & 0x7F);
	ustemp <<= 1;
	if (buf[2] & 0x40)
		ustemp++;

	node->distance = ustemp;

	ustemp = buf[2] & 0x3F;
	ustemp <<= 7;
	ustemp += (buf[3] & 0x7F);
	node->angle = ustemp;

	if (ustemp == 5768)
		return true;
	if (ustemp >= 5760)
		return false;

	return true;
  }



  int
  SelidarDriver::connect (const char * port_path, unsigned int baudrate,
                          unsigned int flag)
  {
    if (isConnected ())
      return Denied;
    
    if (!rxtx)
      return Invalid;
    
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      // establish the serial connection...
      if (!rxtx->bind (port_path, baudrate) || !rxtx->open ())
      {
        return Invalid;
      }
      
      rxtx->flush (0);
    }
    
    connected = true;
    
    return Success;
  }
  
  void
  SelidarDriver::disconnect ()
  {
    if (!connected)
      return;
    stop ();
    cout<<"lidar stop!"<<endl;
    rxtx->close ();
  }
  
  bool
  SelidarDriver::isConnected ()
  {
    return connected;
  }
  


  int
  SelidarDriver::stop ()
  {
	if(!connected)
		return Failure;

	rxtx->senddata((const unsigned char *)LIDAR_CMD_STOP, strlen(LIDAR_CMD_STOP));

    return Success;
  }

  int
  SelidarDriver::startScan ()
  {

    if (!connected)
      return Failure;
    if (scanning)
      return Denied;
    
    stop ();

    if (!connected)
      return Failure;
    rxtx->senddata ((const unsigned char *)LIDAR_CMD_START, strlen(LIDAR_CMD_START));
    {
      boost::mutex::scoped_lock auto_lock (rxtx_lock);
      
      scanning = true;
    }
    
    return Success;
  }

  int
  SelidarDriver::getOnePoint(LaserDataNode *ldn)
  {
	bool decode_rtn;
	int recv_rtn;
	int ldn_cnt = 0;
	int i;
#ifndef _FAKE

	recv_rtn = rxtx->recvdata(buf_rear, 1024);

	if (recv_rtn <= 0) {
		NS_NaviCommon::Duration(0.1).sleep();
		return 0;
	}
	buf_rear += recv_rtn;

	while ((buf_rear - buf_head) >= 4) {
		if (((*((int *) buf_head)) & 0x80808080) != 0x80000000) {
			buf_head++;
			continue;
		}
		decode_rtn = decode(ldn, buf_head);
		if (decode_rtn) {
			ldn++;
			ldn_cnt++;
		}
		buf_head += 4;
	}
	for (i = 0; i < (buf_rear - buf_head); i++) {
		recv_buf[i] = buf_head[i];
	}
	buf_head = recv_buf;
	buf_rear = recv_buf + i;
	return ldn_cnt;
#else
	static double i = 0;
	i += 5.76;
	if (i > 5760.0)
	{
		i = 0.0;
	}
	ldn.angle = int(i);
	ldn.distance = 200;
	return ldn;
#endif
  }
}
