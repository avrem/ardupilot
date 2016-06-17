#pragma once

#include "tinyxml.h"

//typedef float Real;

enum { PID_SYSTEM, 
	   PID_SYSTEM2, 
	   MANUAL,
	   FUZZY_PID, 
	   FUZZY_PID2,
	   FUZZY1,
	   FUZZY2,
	   FUZZY_NEURAL1,
	   FUZZY_NEURAL2		
};

class ElytraConfigurator
{
public:
	ElytraConfigurator();
	~ElytraConfigurator();
	void scanSetupFile();
	void scanBenchFile();
	 bool getOkLoad();

	int getStabilizeSystem();

	 float getPPitch();
	 float getPRoll();
	 float getPYaw();

	 float getIPitch();
	 float getIRoll();
	 float getIYaw();

	 float getDPitch();
	 float getDRoll();
	 float getDYaw();

	 float getPPitchTilt();
	 float getPRollTilt();
	 float getPYawTilt();

	 float getIPitchTilt();
	 float getIRollTilt();
	 float getIYawTilt();

	 float getDPitchTilt();
	 float getDRollTilt();
	 float getDYawTilt();

	 float getIMaxPitch();
	 float getIMaxRoll();
	 float getIMaxYaw();

	 long getC1();
	 long getC2();
	 long getC3();
	 long getC4();

	 long getP1();
	 long getP2();
	 long getP3();
	 long getP4();

private:
	bool loadOk;
	float pp,pr,py,ip,ir,iy,dp,dr,dy;
	float ppt,prt,pyt,ipt,irt,iyt,dpt,drt,dyt;
	long c1,c2,c3,c4,p1,p2,p3,p4;
	long imp,imr,imy;
	void setStabilizeSystem(const char* str);
	int stabSystem;
};

