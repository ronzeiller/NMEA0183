/*
N2kDataToNMEA0183.cpp

Copyright (c) 2015-2018 Timo Lappalainen, Kave Oy, www.kave.fi

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

//#define DEBUGMODE // Prints parsed datas from PGN 129038 and the !AIVDM itself on Serial
#include "N2kDataToNMEA0183.h"
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include <AISMessages.h>
#include <AISUnitsConverts.h>

//*****************************************************************************
void tN2kDataToNMEA0183::HandleMsg(const tN2kMsg &N2kMsg) {
  switch (N2kMsg.PGN) {
    case 127250UL: HandleHeading(N2kMsg);
    case 127258UL: HandleVariation(N2kMsg);
    case 128259UL: HandleBoatSpeed(N2kMsg);
    case 128267UL: HandleDepth(N2kMsg);
    case 129025UL: HandlePosition(N2kMsg);
    case 129026UL: HandleCOGSOG(N2kMsg);
    case 129029UL: HandleGNSS(N2kMsg);
    case 129038UL: HandleAISClassAPosReport(N2kMsg); break; // AIS Class A Position Report
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::Update() {
  SendRMC();
  if ( LastHeadingTime+2000<millis() ) Heading=N2kDoubleNA;
  if ( LastCOGSOGTime+2000<millis() ) { COG=N2kDoubleNA; SOG=N2kDoubleNA; }
  if ( LastPositionTime+4000<millis() ) { Latitude=N2kDoubleNA; Longitude=N2kDoubleNA; }
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendMessage(const tNMEA0183Msg &NMEA0183Msg) {
  if ( pNMEA0183!=0 ) pNMEA0183->SendMessage(NMEA0183Msg);
  if ( SendNMEA0183MessageCallback!=0 ) SendNMEA0183MessageCallback(NMEA0183Msg);
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleHeading(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference ref;
double _Deviation=0;
double _Variation;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kHeading(N2kMsg, SID, Heading, _Deviation, _Variation, ref) ) {
    if ( ref==N2khr_magnetic ) {
      if ( !N2kIsNA(_Variation) ) Variation=_Variation; // Update Variation
      if ( !N2kIsNA(Heading) && !N2kIsNA(Variation) ) Heading-=Variation;
    }
    LastHeadingTime=millis();
    if ( NMEA0183SetHDG(NMEA0183Msg,Heading,_Deviation,Variation) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleVariation(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kMagneticVariation Source;

  ParseN2kMagneticVariation(N2kMsg,SID,Source,DaysSince1970,Variation);
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleBoatSpeed(const tN2kMsg &N2kMsg) {
unsigned char SID;
double WaterReferenced;
double GroundReferenced;
tN2kSpeedWaterReferenceType SWRT;

  if ( ParseN2kBoatSpeed(N2kMsg,SID,WaterReferenced,GroundReferenced,SWRT) ) {
    tNMEA0183Msg NMEA0183Msg;
    double MagneticHeading=( !N2kIsNA(Heading) && !N2kIsNA(Variation)?Heading+Variation: NMEA0183DoubleNA);
    if ( NMEA0183SetVHW(NMEA0183Msg,Heading,MagneticHeading,WaterReferenced) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleDepth(const tN2kMsg &N2kMsg) {
unsigned char SID;
double DepthBelowTransducer;
double Offset;
double Range;

  if ( ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset,Range) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetDPT(NMEA0183Msg,DepthBelowTransducer,Offset) ) {
        SendMessage(NMEA0183Msg);
      }
      if ( NMEA0183SetDBx(NMEA0183Msg,DepthBelowTransducer,Offset) ) {
        SendMessage(NMEA0183Msg);
      }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {

  if ( ParseN2kPGN129025(N2kMsg, Latitude, Longitude) ) {
    LastPositionTime=millis();
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleCOGSOG(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kHeadingReference HeadingReference;
tNMEA0183Msg NMEA0183Msg;

  if ( ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
    LastCOGSOGTime=millis();
    double MCOG=( !N2kIsNA(COG) && !N2kIsNA(Variation)?COG-Variation:NMEA0183DoubleNA );
    if ( HeadingReference==N2khr_magnetic ) {
      MCOG=COG;
      if ( !N2kIsNA(Variation) ) COG-=Variation;
    }
    if ( NMEA0183SetVTG(NMEA0183Msg,COG,MCOG,SOG) ) {
      SendMessage(NMEA0183Msg);
    }
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleGNSS(const tN2kMsg &N2kMsg) {
unsigned char SID;
tN2kGNSStype GNSStype;
tN2kGNSSmethod GNSSmethod;
unsigned char nSatellites;
double HDOP;
double PDOP;
double GeoidalSeparation;
unsigned char nReferenceStations;
tN2kGNSStype ReferenceStationType;
uint16_t ReferenceSationID;
double AgeOfCorrection;

  if ( ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,Latitude,Longitude,Altitude,GNSStype,GNSSmethod,
                    nSatellites,HDOP,PDOP,GeoidalSeparation,
                    nReferenceStations,ReferenceStationType,ReferenceSationID,AgeOfCorrection) ) {
    LastPositionTime=millis();
  }
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendRMC() {
    if ( NextRMCSend<=millis() && !N2kIsNA(Latitude) ) {
      tNMEA0183Msg NMEA0183Msg;
      if ( NMEA0183SetRMC(NMEA0183Msg,SecondsSinceMidnight,Latitude,Longitude,COG,SOG,DaysSince1970,Variation) ) {
        SendMessage(NMEA0183Msg);
      }
      SetNextRMCSend();
    }
}

//*****************************************************************************
// 129038 AIS Class A Position Report
void tN2kDataToNMEA0183::HandleAISClassAPosReport(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kAISRepeat _Repeat;
  uint32_t _UserID;  // MMSI
  double _Latitude;
  double _Longitude;
  bool _Accuracy;
  bool _RAIM;
  uint8_t _Seconds;
  double _COG;
  double _SOG;
  double _Heading;
  double _ROT;
  tN2kAISNavStatus _NavStatus;
  tNMEA0183Msg NMEA0183Msg;
  char _AISClass = 'A';
  uint8_t _MessageType = 1;

  //Serial.print("Type 1 "); Serial.println( N2kMsg.PGN );

  if ( ParseN2kPGN129038(N2kMsg, SID, _Repeat, _UserID, _Latitude, _Longitude, _Accuracy, _RAIM, _Seconds,
                          _COG, _SOG, _Heading, _ROT, _NavStatus ) ) {


    if ( SetAISType1PosReport(NMEA0183Msg, _MessageType, _Repeat, _UserID,
                          _Latitude, _Longitude, _Accuracy, _RAIM,
                          _Seconds, _COG, _SOG, _Heading, _ROT, _NavStatus, _AISClass) ) {

      SendMessage(NMEA0183Msg);

      // *********************  DEBUG  *****************************************
      #ifdef DEBUGMODE
        std::string temp;
        int32_t iTemp;

        (_Repeat >= 0 && _Repeat <= 3)? iTemp = _Repeat : iTemp = 0;
        Serial.print( "REPEAT    = "); Serial.print(_Repeat); Serial.print( " --> "); Serial.println( iTemp );

        (_UserID > 0 && _UserID < 999999999)? iTemp = _UserID : iTemp = 0;
        Serial.print( "UserID    = ");Serial.print(_UserID); Serial.print( " --> "); Serial.println( iTemp );

        (_NavStatus >= 0 && _NavStatus < 15)? iTemp = _NavStatus : iTemp = 15;
        Serial.print( "NavStatus = ");Serial.print(_NavStatus); Serial.print( " --> "); Serial.println( iTemp );

        Serial.print( "ROT rad/s = ");Serial.print(_ROT); Serial.print( " --> ");
        _ROT *= radsToDegMin;
        (_ROT > -128.0 && _ROT < 128.0)? iTemp = aRoundToInt(_ROT) : iTemp = 128;
        Serial.println( iTemp );

        Serial.print( "SOG (m/s) = "); Serial.print( _SOG ); Serial.print( " --> ");
        _SOG *= msTokn;
        (_SOG >= 0.0 && _SOG < 102.3 )? iTemp = aRoundToInt( 10 * _SOG) : iTemp = 1023;
        Serial.println( iTemp );

        (_Accuracy == true)? iTemp = 1 : iTemp = 0;
        Serial.print( "Accuracy  = "); Serial.print(_Accuracy); Serial.print( " --> "); Serial.println( iTemp );

        (_Longitude >= -180.0 && _Longitude <= 180.0)? iTemp = aRoundToInt(_Longitude * 600000) : iTemp = 181 * 600000;
        //temp = std::bitset<28>( iTemp ).to_string();
        Serial.print("Longitude = ");Serial.print(_Longitude );  Serial.print( " --> "); Serial.println( iTemp );
        //Serial.print( " -->  "); Serial.println( temp.c_str() );

        (_Latitude >= -90.0 && _Latitude <= 90.0)? iTemp = aRoundToInt(_Latitude * 600000) : iTemp = 91 * 600000;
        //temp = std::bitset<27>( iTemp ).to_string();
        Serial.print("Latitude  = ");Serial.print(_Latitude );  Serial.print( " --> "); Serial.println( iTemp );
        //Serial.print( " --> "); Serial.println( temp.c_str() );

        Serial.print( "COG "); Serial.print( _COG ); Serial.print( " rad --> ");
        _COG *= radToDeg;
        (_COG >= 0.0 && _COG < 360 )? iTemp = aRoundToInt(10 * _COG) : iTemp = 3600;
        //temp =  std::bitset<12>(iTemp).to_string();
        Serial.print( _COG ); Serial.print( "° --> "); Serial.println( iTemp );
        //Serial.print( " --> "); Serial.println( temp.c_str() );

        Serial.print( "HDG    "); Serial.print( _Heading ); Serial.print( "rad --> ");
        _Heading *= radToDeg;
        (_Heading >= 0.0 && _Heading <= 359.0 )? iTemp = aRoundToInt( _Heading ) : iTemp = 511;
        Serial.print( _Heading ); Serial.print( " --> "); Serial.println( iTemp );

        Serial.print( "RAIM      = ");Serial.print(_RAIM);Serial.println("");

        Serial.print(NMEA0183Msg.Prefix);
        Serial.print(NMEA0183Msg.Sender());
        Serial.print(NMEA0183Msg.MessageCode());
        for (int i=0; i<NMEA0183Msg.FieldCount(); i++) {
          Serial.print(",");
          Serial.print(NMEA0183Msg.Field(i));
        }
        char buf[7];
        sprintf(buf,"*%02X\r\n",NMEA0183Msg.CheckSum);
        Serial.print(buf);
        Serial.print("\r\n");
        Serial.println("–––––––––––––––––––––––––––––––––––––––––––––––––––––");
      #endif

    } else {
      #ifdef DEBUGMODE
        Serial.println("Did not work!!");
        #endif
    }
  }
}
