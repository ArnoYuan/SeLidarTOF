/*
 * SelidarApplication.cpp
 *
 *  Created on: 2016年10月12日
 *      Author: lichq
 */

#include "SelidarApplication.h"
#include <Console/Console.h>
//for debugging
#include <assert.h>
#include <Time/Utils.h>
#include <Parameter/Parameter.h>

using std::cin;
using std::cout;
using std::endl;

namespace NS_Selidar
{
  
#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif
  
  SelidarApplication::SelidarApplication ()
  {
    //publisher = new NS_DataSet::Publisher<NS_DataType::LaserScan> ("LASER_SCAN");
    //serial_baudrate = 115200;
    serial_baudrate = 230400;
    frame_id = "laser_frame";
    scan_count = 0;
    scan_timeout = 3;
    inverted = false;
  }
  
  SelidarApplication::~SelidarApplication ()
  {
    scan_count = 0;
    //delete publisher;
  }
  
  void
  SelidarApplication::loadParameters ()
  {
	NS_NaviCommon::Parameter parameter;
    parameter.loadConfigurationFile ("selidar.xml");
    serial_port = parameter.getParameter ("serial_port", "/dev/ttyUSB0");
    //serial_baudrate = parameter.getParameter ("serial_baudrate", 115200);
    serial_baudrate = parameter.getParameter("serial_baudrate", 230400);
    frame_id = parameter.getParameter ("frame_id", "laser_frame");
    align_number = parameter.getParameter("align_number", 360);
    if (parameter.getParameter ("inverted", 0) == 1)
    {
      inverted = true;
    }
    else
    {
      inverted = false;
    }
  }

  int
  SelidarApplication::publishScanData(
    LidarConstant ldConst,
    NS_NaviCommon::Time start_scan_time,
	double scan_duration,
	LaserDataNode *node,
	int actual_cnt,
	int align_cnt
  )
  {
	  NS_DataType::LaserScan scan_msg;
	  scan_msg.header.frame_id = frame_id;
	  scan_msg.header.stamp = start_scan_time;
	  scan_msg.angle_min = ldConst.angle_min;
	  scan_msg.angle_max = ldConst.angle_max;
	  scan_msg.angle_increment = ldConst.angle_increment;
	  scan_msg.scan_time = scan_duration;
	  scan_msg.time_increment = scan_duration/align_cnt;
	  scan_msg.range_min = ldConst.range_min;
	  scan_msg.range_max = ldConst.range_max;
	  scan_msg.ranges.resize(align_cnt);
	  scan_msg.intensities.resize(align_cnt);

	  int j;
	  int j_min = 0;
	  int j_max = 0;
	  int index;
	  int offset = align_cnt/2;
	  for(int i=0;i<align_cnt;i++)
	  {
		  j_min=j_max;
		  j_max = (i+1)*actual_cnt/align_cnt-1;
		  for(j=j_min;j<j_max;j++)
		  {
			  if(inverted)
			  {
				  index = align_cnt-offset+i;
				  if(index>=align_cnt)
					  index = index-align_cnt;
			  }
			  else
			  {
				  index = align_cnt-offset-i;
				  if(index<0)
				  {
					  index = align_cnt +index;
				  }
			  }

			  if((node+j)->distance>1&&(node+j)->distance<3500)
			  {
				  scan_msg.ranges[index]=(node+j)->distance*ldConst.kDistance;
				  scan_msg.intensities[index] = 47.0;
				  break;
			  }
			  scan_msg.ranges[index]=std::numeric_limits<float>::infinity();
			  scan_msg.intensities[index]=0.0;
		  }
	  }
	  console.message("publish scan data:");
	  console.message("start time:%f, duration:%f\r\n", scan_msg.header.stamp.toSec(),scan_msg.scan_time);
	  unsigned short i=0;
	  for(i=0;i<scan_msg.ranges.size();i++)
	  {
		  cout<<scan_msg.ranges[i]<<",";
	  }
	  cout<<endl;

	  //publisher->publish (scan_msg);
	  return 0;
  }

  void
  SelidarApplication::scanLoop ()
  {
    NS_NaviCommon::Time start_scan_time=NS_NaviCommon::Time::now();
    double scan_duration;
    drv.init(align_number);
    LaserDataNode *ldn_array = new LaserDataNode[5760];
    memset((void *)ldn_array, 0,5760*sizeof(LaserDataNode));
    LaserDataNode *ldn = new LaserDataNode[256];
    LaserDataNode *ldn_tmp;
    int ldn_cnt = 0;

    int last_anl = -1;

    drv.startScan ();
    int count = 0;
    while (running)
    {
      ldn_cnt = drv.getOnePoint(ldn);
      for(int i=0;i<ldn_cnt;i++)
      {
    	  ldn_tmp = ldn+i;
    	  if(ldn_tmp->angle==5768)
    	  {
    		  scan_duration=(NS_NaviCommon::Time::now()-start_scan_time).toSec()*0.001;
    		  publishScanData(drv.ldConst,
    				  start_scan_time,
					  scan_duration,
					  ldn_array,
					  5760,
					  align_number);
    		  memset((void *)ldn_array, 0, 5760*sizeof(LaserDataNode));
    		  last_anl = -1;
    		  start_scan_time = NS_NaviCommon::Time::now();
    		  count = 0;
    	  }
    	  else
    	  {
//    		  printf("count=%d\r\n", count);
//    		  count++;
    		  if(ldn_tmp->angle>last_anl)
    		  {
    			  (ldn_array+ldn_tmp->angle)->angle=ldn_tmp->angle;
    			  (ldn_array+ldn_tmp->angle)->distance = ldn_tmp->distance;
    			  last_anl = ldn_tmp->angle;
    		  }
    	  }
      }
    }

  }
  
  void
  SelidarApplication::run ()
  {
    console.message ("selidar is running!");
    
    loadParameters ();

    // make connection...
    if (IS_FAIL(
        drv.connect (serial_port.c_str (), (unsigned int )serial_baudrate, 0)))
    {
      console.error (
          "cannot bind to the specified serial port %s.", serial_port.c_str ());
    }

    NS_NaviCommon::delay (100);

    running = true;
    
    scan_thread = boost::thread (
        boost::bind (&SelidarApplication::scanLoop, this));
  }
  
  void
  SelidarApplication::quit ()
  {
    console.message ("selidar is quitting!");
    
    running = false;
    
    scan_thread.join ();

    drv.disconnect ();
  }

}

