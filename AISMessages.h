/*
AISMessages.h

Copyright (c) 2019 Ronnie Zeiller

Based on the works of Timo Lappalainen and Eric S. Raymond and Kurt Schwehr https://gpsd.gitlab.io/gpsd/AIVDM.html

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef _tAISMessages_H_
#define _tAISMessages_H_

#include <stdio.h>
#include <time.h>
#include <NMEA0183Msg.h>
#include <unordered_map>
#include <bitset>

#ifndef Arduino
typedef uint8_t byte;
#endif

#ifndef
#define radToDeg(x)   (x) * 360.0 / (2 * M_PI)
#endif

//*****************************************************************************
// Types 1, 2 and 3: Position Report Class A
//std::string SetAISClassAPosReport(tAISMsg &AISMsg, uint8_t MessageType, uint8_t Repeat,
bool SetAISClassAPosReport(tNMEA0183Msg &NMEA0183Msg, uint8_t MessageType, uint8_t Repeat,
													uint32_t UserID, double Latitude, double Longitude, bool Accuracy, bool RAIM, uint8_t Seconds,
													double COG, double SOG, double Heading, double ROT, uint8_t NavStatus, const char *Src="");

// ****************  Helper for AIS  ***********************************
inline std::string intToBinary( int n ) { std::string binary = std::bitset<6>(n).to_string(); return binary; }
inline double AISRadToDeg(double x)  { return ( (x) * 360.0 / (2 * M_PI) ); }
std::string strToBinary(std::string s);
std::string convertBinaryAISPayloadToAscii( std::string bPayload, uint8_t part, uint8_t total );

#endif
