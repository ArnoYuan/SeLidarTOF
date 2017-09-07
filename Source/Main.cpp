/*
 * Main.cpp
 *
 *  Created on: 2016年8月17日
 *      Author: seeing
 */

#include "SelidarApplication.h"

using namespace NS_Selidar;

int
main (int argc, char* argv[])
{
  SelidarApplication app;
  
  app.initialize ();
  
  app.run ();
  
  app.pending ();
  
  return 0;
}

