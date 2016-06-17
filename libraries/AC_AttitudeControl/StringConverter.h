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

//Changed.
#include "tinyxml.h"
#ifndef __StringConverter_H__
#define __StringConverter_H__

typedef float Real;

class StringConverter
{
    public:


		static const char* getAttrib(TiXmlElement *XMLNode, const char* parameter, const char* defaultValue = "");
		static Real getAttribReal(TiXmlElement *XMLNode, const char* parameter, Real defaultValue = 0);
		static bool getAttribBool(TiXmlElement *XMLNode, const char* parameter, bool defaultValue = false);
		static long getAttribLong(TiXmlElement *XMLNode, const char* parameter, long defaultValue = 0);

        static Real parseReal(const char* val);
        /** Converts a String to a Angle. 
        @returns
            0.0 if the value could not be parsed, otherwise the Angle version of the String.
        */
       
        static int parseInt(const char* val);
        /** Converts a String to a whole number. 
        @returns
            0.0 if the value could not be parsed, otherwise the numeric version of the String.
        */
        static unsigned int parseUnsignedInt(const char*  val);
        /** Converts a String to a whole number. 
        @returns
            0.0 if the value could not be parsed, otherwise the numeric version of the String.
        */
        static long parseLong(const char*  val);
        /** Converts a String to a whole number. 
        @returns
            0.0 if the value could not be parsed, otherwise the numeric version of the String.
        */
        static unsigned long parseUnsignedLong(const char*  val);
        /** Converts a String to a boolean. 
        @remarks
            Returns true if case-insensitive match of the start of the string
			matches "true", "yes" or "1", false otherwise.
        */
        static bool parseBool(const char*  val);
		/** Parses a Vector2 out of a String.
        @remarks
            Format is "x y" ie. 2 Real components, space delimited. Failure to parse returns
            Vector2::ZERO.
        */
};




#endif

