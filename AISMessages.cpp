/*
AISMessages.cpp

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

#include <AISMessages.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <bitset>
#include <unordered_map>
#include <sstream>

#if !defined(roundToInt)
uint8_t roundToInt(double val) {
  return val >= 0
      ? (int) floor(val + 0.5)
      : (int) ceil(val - 0.5);
}
#endif

std::unordered_map<char, int> ac = {
							{'@',  0}, {'A', 1}, {'B', 2}, {'C', 3}, {'D', 4}, {'E', 5}, {'F', 6}, {'G', 7}, {'H', 8}, {'I', 9},
							{'J', 10}, {'K', 11}, {'L', 12}, {'M', 13}, {'N', 14}, {'O', 15}, {'P', 16}, {'Q', 17}, {'R', 18}, {'S', 19},
							{'T', 20}, {'U', 21}, {'V', 22}, {'W', 23}, {'X', 24}, {'Y', 25}, {'Z', 26}, {'[', 27}, {'\\', 28}, {']', 29},
							{'^', 30}, {'_', 31}, {' ', 32}, {'!', 33}, {'\"', 34}, {'#', 35}, {'$', 36}, {'%', 37}, {'&', 38}, {'\'', 39},
							{'(', 40}, {')', 41}, {'*', 42}, {'+', 43}, {',', 44}, {'-', 45}, {'.', 46}, {'/', 47}, {'0', 48}, {'1', 49},
							{'2', 50}, {'3', 51}, {'4', 52}, {'5', 53}, {'6', 54}, {'7', 55}, {'8', 56}, {'9', 57}, {':', 58}, {';', 59},
							{'<', 60}, {'=', 61}, {'>', 62}, {'?', 63}
							};

//*****************************************************************************
// Types 1, 2 and 3: Position Report Class A  -> https://gpsd.gitlab.io/gpsd/AIVDM.html
// total of 168 bits, occupying one AIVDM sentence
// Example: !AIVDM,1,1,,A,133m@ogP00PD;88MD5MTDww@2D7k,0*46
// Payload: Payload: 133m@ogP00PD;88MD5MTDww@2D7k
// Message type 1 has a payload length of 168 bits.
// because AIS encodes messages using a 6-bits ASCII mechanism and 168 divided by 6 is 28.
//
// Got values from:
// bool ParseN2kPGN129038(const tN2kMsg &N2kMsg, uint8_t &MessageID, tN2kAISRepeat &Repeat,
//												uint32_t &UserID, double &Latitude, double &Longitude,
//    										bool &Accuracy, bool &RAIM, uint8_t &Seconds, double &COG,
//												double &SOG, double &Heading, double &ROT, tN2kAISNavStatus &NavStatus);
//std::string SetAISClassAPosReport(tAISMsg &AISMsg, uint8_t MessageType, uint8_t Repeat,
bool SetAISClassAPosReport(tNMEA0183Msg &NMEA0183Msg, uint8_t MessageType, uint8_t Repeat,
													uint32_t UserID, double Latitude, double Longitude, bool Accuracy, bool RAIM, uint8_t Seconds,
													double COG, double SOG, double Heading, double ROT, uint8_t NavStatus, const char *Src) {

  std::string payload;
  int32_t iTemp;

  // AIS Type 1 Message
  // Fields	Len	Description
  // ––––––––––––––––––––––
	// 0-5	  6		Message Type	Constant 1-3
	// $enc .= str_pad(decbin( $this->ado->TYPE ), 6, '0', STR_PAD_LEFT);
  (MessageType >= 0 && MessageType <= 3  )? iTemp = MessageType : iTemp = 1;
  payload.append( std::bitset<6>(iTemp).to_string() );

  // Repeat Indicator: 0 = default; 3 = do not repeat any more
  (Repeat >= 0 && Repeat <= 3)? iTemp = Repeat : iTemp = 0;
  payload.append( std::bitset<2>(iTemp).to_string() );

  // 8-37	30		MMSI				9 decimal digits
  (UserID > 0 && UserID < 999999999)? iTemp = UserID : iTemp = 0;
  payload.append( std::bitset<30>(iTemp).to_string() );

	// 38-41	4		Navigational Status	e.g.: "Under way sailing"
  (NavStatus >= 0 && NavStatus < 15) ? iTemp = NavStatus : iTemp = 15;
  payload.append( std::bitset<4>(iTemp).to_string() );

  // 42-49	8		Rate of Turn ROT	rad/s -> degrees per minute  128 = N/A
  /*
    0 = not turning
    1…126 = turning right at up to 708 degrees per minute or higher
    1…-126 = turning left at up to 708 degrees per minute or higher
    127 = turning right at more than 5deg/30s (No TI available)
    -127 = turning left at more than 5deg/30s (No TI available)
    128 (80 hex) indicates no turn information available (default)
  */
  ROT = AISRadToDeg(ROT) * 60;
  (ROT > -128.0 && ROT < 128.0)? iTemp = roundToInt(ROT) : iTemp = 128;
  payload.append( std::bitset<8>(iTemp).to_string() );

  // 50-59	10	SOG m/s -> Knots with one digit	x10, 1023 = N/A
  SOG *= 3600.0 / 1852.0;
  (SOG >= 0.0 && SOG < 102.3 )? iTemp = roundToInt( 10 * SOG) : iTemp = 1023;
  payload.append( std::bitset<10>(iTemp).to_string() );

  // 60	1		GPS Accuracy 1 oder 0
  (Accuracy == true)? iTemp = 1 : iTemp = 0;
  payload.append(std::bitset<1>(iTemp).to_string() );

  // 61-88	28		Longitude in Minutes / 10000
  // Longitude is given in in 1/10000 min; divide by 600000.0 to obtain degrees.
  // Values up to plus or minus 180 degrees, East = positive, West \= negative.
  // A value of 181 degrees (0x6791AC0 hex) indicates that longitude is not available and is the default.
  (Longitude >= -180.0 && Longitude <= 180.0)? iTemp = (int) (Longitude * 600000) : iTemp = 181 * 600000;
  payload.append( std::bitset<28>(iTemp).to_string() );

  // 89 -> 115	27		Latitude in Minutes / 10000
  // Values up to plus or minus 90 degrees, North = positive, South = negative.
  // A value of 91 degrees (0x3412140 hex) indicates latitude is not available and is the default.
  (Latitude >= -90.0 && Latitude <= 90.0)? iTemp = (int) (Latitude * 600000) : iTemp = 91 * 600000;
  payload.append( std::bitset<27>(iTemp).to_string() );

  // COG: 116 - 127 | 12  Course over ground will be 3600 (0xE10) if that data is not available.
  COG = AISRadToDeg(COG);
  (COG >= 0.0 && COG < 360 )? iTemp = roundToInt(10 * COG) : iTemp = 3600;
  payload.append( std::bitset<12>(iTemp).to_string() );

  // 128 -136		9		True Heading (HDG) rad -> 0 to 359 degrees, 511 = not available.
  Heading = AISRadToDeg(Heading);
  (Heading >= 0.0 && Heading <= 359.0 )? iTemp = roundToInt( Heading ) : iTemp = 511;
  payload.append( std::bitset<9>(iTemp).to_string() );

  // 137 - 142	6	 Seconds in UTC timestamp should be 0-59, except for these special values:
  // 60 if time stamp is not available (default)
  // 61 if positioning system is in manual input mode
  // 62 if Electronic Position Fixing System operates in estimated (dead reckoning) mode,
  // 63 if the positioning system is inoperative.
  (Seconds >= 0 && Seconds <= 63 )? iTemp = Seconds : iTemp = 60;
  payload.append( std::bitset<6>(iTemp).to_string() );

  // 143 - 144	2		Maneuver Indicator: 0 (default) 1, 2  (not delivered within this PGN)
  payload.append( std::bitset<2>(0).to_string() );

  // 145-147  3 spare
  payload.append( std::bitset<3>(0).to_string() );

  // 148 - 148	1		RAIM flag 0 = RAIM not in use (default), 1 = RAIM in use
  (RAIM == true)? iTemp = 1 : iTemp = 0;
  payload.append(std::bitset<1>(iTemp).to_string() );

  // 149 - 167	19		Radio Status
  payload.append( std::bitset<19>(0).to_string() );

  // convert 6-bit binary payload to ASCI encoded payload
  payload = convertBinaryAISPayloadToAscii(payload, 1,1);

  char ais[payload.size() + 1];
  payload.copy(ais, payload.size() + 1);
  ais[payload.size()] = '\0';

  char _Prefix='!';
  if ( !NMEA0183Msg.Init("VDM","AI", _Prefix)) return false;
  if ( !NMEA0183Msg.AddStrField("1")) return false;
  if ( !NMEA0183Msg.AddStrField("1")) return false;
  if ( !NMEA0183Msg.AddEmptyField()) return false;
  if ( !NMEA0183Msg.AddStrField("A")) return false;
  if ( !NMEA0183Msg.AddStrField( ais )) return false;

  return true;
}

// *****************************************************************************
// needed for Type 5: Static and Voyage Related Data
std::string strToBinary(std::string s) {
  std::string binary_outputInformations;
  for (std::size_t i = 0; i < s.size(); ++i) {
    std::bitset<6> b(s.c_str()[i]);
      binary_outputInformations+= b.to_string();
  }
  return binary_outputInformations;
}

// *****************************************************************************
std::string convertBinaryAISPayloadToAscii( std::string bPayload, uint8_t part, uint8_t total ) {
  std::string itu;

  uint32_t len      = bPayload.size();
  uint16_t rem6     = len % 6;
  uint32_t pad6_len = 0;
  if (rem6) {
    pad6_len = 6 - rem6;  // missing '0'
    bPayload.insert(0, pad6_len, '0');
  }
  len = bPayload.size();
  // if ( len % 6 ) return NULL;   // Something went wrong, return empty string
  len = len / 6;

  uint32_t offset;
  std::string s;
  uint8_t dec;
  for ( int i=0; i<len; i++ ) {
    offset = i * 6;
    s = bPayload.substr(offset, 6);
    dec = strtoull (s.c_str(), NULL, 2);  //binToDec
    if (dec < 40 ) dec += 48;
    else dec += 56;
    char c = dec;
    itu += c;
  }
  return itu;
}
