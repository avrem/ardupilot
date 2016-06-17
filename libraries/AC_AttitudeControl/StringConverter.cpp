/*
-----------------------------------------------------------------------------
This source file is part of OGRE
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2006 Torus Knot Software Ltd
Also see acknowledgements in Readme.html

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.

You may alternatively use this source under the terms of a specific version of
the OGRE Unrestricted License provided you have obtained such a license from
Torus Knot Software Ltd.
-----------------------------------------------------------------------------
*/
#include "StringConverter.h"


typedef float Real;

    //-----------------------------------------------------------------------
    Real StringConverter::parseReal(const char*  val)
    {
		// Use istringstream for direct correspondence with toString
		//std::stringstream str(val);
		Real ret = 0;
		//str >> ret;
		ret = atof(val);
        return ret;
    }
    //-----------------------------------------------------------------------
    int StringConverter::parseInt(const char*  val)
    {
		// Use istringstream for direct correspondence with toString
		//std::stringstream str(val);
		int ret = 0;
		//str >> ret;
		ret = atoi(val);
        return ret;
    }
    //-----------------------------------------------------------------------
    unsigned int StringConverter::parseUnsignedInt(const char*  val)
    {
		// Use istringstream for direct correspondence with toString
		//std::stringstream str(val);
		unsigned int ret = 0;
		//str >> ret;
		ret = atoi(val);
		return ret;
    }
    //-----------------------------------------------------------------------
    long StringConverter::parseLong(const char*  val)
    {
		// Use istringstream for direct correspondence with toString
		//std::stringstream str(val);
		long ret = 0;
		//str >> ret;
		ret = atol(val);
		return ret;
    }
    //-----------------------------------------------------------------------
    unsigned long StringConverter::parseUnsignedLong(const char*  val)
    {
		// Use istringstream for direct correspondence with toString
		//std::stringstream str(val);
		unsigned long ret = 0;
		//str >> ret;
		ret = atol(val);
		return ret;
    }
    //-----------------------------------------------------------------------
    bool StringConverter::parseBool(const char*  val)
    {
		return (val==(char*)"true");
    }


const char* StringConverter::getAttrib(TiXmlElement *XMLNode, const char* parameter, const char* defaultValue)
{
	if (XMLNode->Attribute(parameter))
		return XMLNode->Attribute(parameter);
	else
		return defaultValue;
}

Real StringConverter::getAttribReal(TiXmlElement *XMLNode, const char* parameter, Real defaultValue)
{
	return StringConverter::parseReal(getAttrib(XMLNode,parameter,(char*)"0"));
}

long StringConverter::getAttribLong(TiXmlElement *XMLNode, const char* parameter, long defaultValue)
{
	return StringConverter::parseLong(getAttrib(XMLNode,parameter,(char*)"0"));
}

bool StringConverter::getAttribBool(TiXmlElement *XMLNode, const char* parameter, bool defaultValue)
{
	return getAttrib(XMLNode,parameter,(char*)"false")==(char*)"true";
}