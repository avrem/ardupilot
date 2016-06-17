#include "ElytraConfigurator.h"
#include "StringConverter.h"



ElytraConfigurator::ElytraConfigurator()
{
	stabSystem=0;
	pp=pr=py=ip=ir=iy=dp=dr=dy=0;
	ppt=prt=pyt=ipt=irt=iyt=dpt=drt=dyt=0;
	imp=imr=imy=0;
	c1=c2=c3=c4=p1=p2=p3=p4=0;
	loadOk=false;
}


ElytraConfigurator::~ElytraConfigurator()
{

}

/*void ElytraConfigurator::dump_to_stdout(TiXmlNode* pParent, unsigned int indent)
{
	
}*/

void ElytraConfigurator::setStabilizeSystem(const char* str)
{
	
}

void ElytraConfigurator::scanSetupFile()
{
	//	TiXmlDocument   *SequenceDoc;
	//TiXmlElement   *SequenceRoot;
	TiXmlDocument doc("elytra.xml");

		bool loadOkay = doc.LoadFile();
		if (loadOkay)
		{
			//printf("Parsing settings from: %s:\n", "elytra.xml");
			TiXmlElement* pElem = doc.FirstChildElement("params");
			if (pElem)
			{
				pElem = pElem->FirstChildElement("stab");
				if (pElem)
				{
					const char* str =  StringConverter::getAttrib(pElem, (char*)"type", (char*)"PID_SYSTEM");

					/*std::string str =  StringConverter::getAttrib(pElem, "type", "PID_SYSTEM");*/
					if (str==(char*)"PID_SYSTEM")
						stabSystem=PID_SYSTEM;
					if (str==(char*)"PID_SYSTEM2")
						stabSystem=PID_SYSTEM2;
					if (str==(char*)"MANUAL")
						stabSystem=MANUAL;
					if (str==(char*)"FUZZY_PID")
						stabSystem=FUZZY_PID;
					if (str==(char*)"FUZZY_PID2")
						stabSystem=FUZZY_PID2;
					if (str==(char*)"FUZZY1")
						stabSystem=FUZZY1;
					if (str==(char*)"FUZZY2")
						stabSystem=FUZZY2;
					if (str==(char*)"FUZZY_NEURAL1")
						stabSystem=FUZZY_NEURAL1;
					if (str==(char*)"FUZZY_NEURAL2")
						stabSystem=FUZZY_NEURAL2;
					//setStabilizeSystem(type.c_str());
					//std::cout << type.c_str() << std::endl;
				}
				pElem = doc.FirstChildElement("params");
				pElem = pElem->FirstChildElement("pitch");
				if (pElem)
				{
					pp=StringConverter::getAttribReal(pElem, (char*)"P", 0.0f);
					ip=StringConverter::getAttribReal(pElem, (char*)"I", 0.0f);
					dp=StringConverter::getAttribReal(pElem, (char*)"D", 0.0f);
					ppt=StringConverter::getAttribReal(pElem, (char*)"P2", 0.0f);
					ipt=StringConverter::getAttribReal(pElem, (char*)"I2", 0.0f);
					dpt=StringConverter::getAttribReal(pElem, (char*)"D2", 0.0f);
					imp=StringConverter::getAttribLong(pElem, (char*)"IMAX", 350000);
				}
				pElem = doc.FirstChildElement("params");
				pElem = pElem->FirstChildElement("roll");
				if (pElem)
				{
					pr=StringConverter::getAttribReal(pElem, (char*)"P", 0.0f);
					ir=StringConverter::getAttribReal(pElem, (char*)"I", 0.0f);
					dr=StringConverter::getAttribReal(pElem, (char*)"D", 0.0f);
					prt=StringConverter::getAttribReal(pElem, (char*)"P2", 0.0f);
					irt=StringConverter::getAttribReal(pElem, (char*)"I2", 0.0f);
					drt=StringConverter::getAttribReal(pElem, (char*)"D2", 0.0f);
					imr=StringConverter::getAttribLong(pElem, (char*)"IMAX", 350000);
				}
				pElem = doc.FirstChildElement("params");
				pElem = pElem->FirstChildElement("yaw");
				if (pElem)
				{
					py=StringConverter::getAttribReal(pElem, (char*)"P", 0.0f);
					iy=StringConverter::getAttribReal(pElem, (char*)"I", 0.0f);
					dy=StringConverter::getAttribReal(pElem, (char*)"D", 0.0f);
					pyt=StringConverter::getAttribReal(pElem, (char*)"P2", 0.0f);
					iyt=StringConverter::getAttribReal(pElem, (char*)"I2", 0.0f);
					dyt=StringConverter::getAttribReal(pElem, (char*)"D2", 0.0f);
					imy=StringConverter::getAttribLong(pElem, (char*)"IMAX", 350000);
				}
			}
			loadOk=true;
		}
		else
		{
			loadOk=false;
			//printf("Failed to load file \"%s\"\n", "elytra.xml");
		}
	

}

void ElytraConfigurator::scanBenchFile()
{
	//	TiXmlDocument   *SequenceDoc;
	//TiXmlElement   *SequenceRoot;
	TiXmlDocument doc("bench.xml");

		bool loadOkay = doc.LoadFile();
		if (loadOkay)
		{
			//printf("Parsing settings from: %s:\n", "elytra.xml");
			TiXmlElement* pElem = doc.FirstChildElement("params");
			if (pElem)
			{
				pElem = pElem->FirstChildElement("bench");
				
				if (pElem)
				{
					c1=StringConverter::getAttribLong(pElem, (char*)"c1", 0.0f);
					c2=StringConverter::getAttribLong(pElem, (char*)"c2", 0.0f);
					c3=StringConverter::getAttribReal(pElem, (char*)"c3", 0.0f);
					c4=StringConverter::getAttribReal(pElem, (char*)"c4", 0.0f);
					
					p1=StringConverter::getAttribLong(pElem, (char*)"p1", 0.0f);
					p2=StringConverter::getAttribLong(pElem, (char*)"p2", 0.0f);
					p3=StringConverter::getAttribReal(pElem, (char*)"p3", 0.0f);
					p4=StringConverter::getAttribReal(pElem, (char*)"p4", 0.0f);
				}
				
			}
			loadOk=true;
		}
		else
		{
			loadOk=false;
			//printf("Failed to load file \"%s\"\n", "elytra.xml");
		}
	

}



int ElytraConfigurator::getStabilizeSystem()
{
	return stabSystem;
}

Real ElytraConfigurator::getPPitch(){return pp;}
Real ElytraConfigurator::getPRoll(){return pr;}
Real ElytraConfigurator::getPYaw(){return py;}

Real ElytraConfigurator::getIPitch(){return ip;}
Real ElytraConfigurator::getIRoll(){return ir;}
Real ElytraConfigurator::getIYaw(){return iy;}

Real ElytraConfigurator::getDPitch(){return dp;}
Real ElytraConfigurator::getDRoll(){return dr;}
Real ElytraConfigurator::getDYaw(){return dy;}

Real ElytraConfigurator::getPPitchTilt(){return ppt;}
Real ElytraConfigurator::getPRollTilt(){return prt;}
Real ElytraConfigurator::getPYawTilt(){return pyt;}

Real ElytraConfigurator::getIPitchTilt(){return ipt;}
Real ElytraConfigurator::getIRollTilt(){return irt;}
Real ElytraConfigurator::getIYawTilt(){return iyt;}

Real ElytraConfigurator::getDPitchTilt(){return dpt;}
Real ElytraConfigurator::getDRollTilt(){return drt;}
Real ElytraConfigurator::getDYawTilt(){return dyt;}

Real ElytraConfigurator::getIMaxPitch(){return imp;}
Real ElytraConfigurator::getIMaxRoll(){return imr;}
Real ElytraConfigurator::getIMaxYaw(){return imy;}

bool ElytraConfigurator::getOkLoad(){return loadOk;}

long ElytraConfigurator::getC1(){return c1;}
long ElytraConfigurator::getC2(){return c2;}
long ElytraConfigurator::getC3(){return c3;}
long ElytraConfigurator::getC4(){return c4;}

long ElytraConfigurator::getP1(){return p1;}
long ElytraConfigurator::getP2(){return p2;}
long ElytraConfigurator::getP3(){return p3;}
long ElytraConfigurator::getP4(){return p4;}
