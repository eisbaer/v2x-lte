/*
 * Copyright (c) 2009  EURECOM, DLR, EU FP7 iTETRIS project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Fatma HRIZI <hrizi@eurecom.fr>, Matthias ROECKL <matthias.roeckl@dlr.de>
 */


#include "gn-common-header.h"
#include "ns3/simulator.h"


static const double DEG_TO_RAD = 0.017453292519943295769236907684886;
/// @brief Earth's quatratic mean radius for WGS-84
static const double EARTH_RADIUS_IN_METERS = 6372797.560856;



namespace ns3 {

TypeId
GnCommonHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GnCommonHeader")
    .SetParent<Header> ()
    .AddConstructor<GnCommonHeader> ()
  ;
  return tid;
}

TypeId
GnCommonHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
GnCommonHeader::GetSerializedSize (void) const
{
  /**
  * This is the size of a C2C Common Header
  */
  return 9*4;
}

void
GnCommonHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

   i.WriteU8 ((m_version << 4)|m_nHeader);
   i.WriteU8 ((m_hType << 4)|m_hSubType);
   i.WriteU8 (m_reserved);
   i.WriteU8 (m_flags);
   i.WriteHtonU16 (m_pLength);
   i.WriteU8 (m_tClass);
   i.WriteU8 (m_hLimit);
   i.WriteHtonU64 (m_sourposvector.gnAddr);
   i.WriteHtonU32 (m_sourposvector.Ts);
   i.WriteHtonU32 (m_sourposvector.Lat);
   i.WriteHtonU32 (m_sourposvector.Lon);
   i.WriteHtonU16 (m_sourposvector.Speed);
   i.WriteHtonU16 (m_sourposvector.Heading);
   i.WriteHtonU16 (m_sourposvector.Alt);
   i.WriteU8 (m_sourposvector.TimeAcc << 4 | m_sourposvector.PosAcc);
   i.WriteU8 (m_sourposvector.SpeedAcc << 5 | (m_sourposvector.HeadingAcc << 2) | m_sourposvector.AltAcc);
}

uint32_t
GnCommonHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  uint8_t m_version_nheader = i.ReadU8();   // this byte includes Version (4bit) and nhtype (4bit)
  m_version = (m_version_nheader >> 4)&15;
  m_nHeader = m_version_nheader&15;
  uint8_t m_ht_hst = i.ReadU8();   // this byte includes HT (4bit) and HST (4bit)
  m_hType = (m_ht_hst >> 4)&15;
  m_hSubType = m_ht_hst&15;
  m_reserved = i.ReadU8 ();
  m_flags = i.ReadU8 ();
  m_pLength = i.ReadNtohU16 ();
  m_tClass = i.ReadU8 ();
  m_hLimit = i.ReadU8 ();
  m_sourposvector.gnAddr = i.ReadNtohU64 ();
  m_sourposvector.Ts = i.ReadNtohU32 ();
  m_sourposvector.Lat = i.ReadNtohU32 ();
  m_sourposvector.Lon = i.ReadNtohU32 ();
  m_sourposvector.Speed = i.ReadNtohU16 ();
  m_sourposvector.Heading = i.ReadNtohU16 ();
  m_sourposvector.Alt = i.ReadNtohU16 ();
  uint8_t m_timac_posac = i.ReadU8 ();   // this byte includes TimAc (4bit) and PosAc (4bit)
  m_sourposvector.TimeAcc = (m_timac_posac >> 4)&15;
  m_sourposvector.PosAcc = m_timac_posac&15;
  uint8_t m_sac_hac_aac = i.ReadU8 ();   // this byte includes SAc (3bit), HAc (3bit) and AAc (2bit)
  m_sourposvector.SpeedAcc = (m_sac_hac_aac >> 5)&7;
  m_sourposvector.HeadingAcc = (m_sac_hac_aac >> 2)&7;
  m_sourposvector.AltAcc = m_sac_hac_aac&3;

  return GetSerializedSize (); // The number of bytes consumed.
}

void
GnCommonHeader::Print (std::ostream &os) const
{
 os << "****GN Common Header**** "
         << "source ID: " << m_sourposvector.gnAddr << " "
         << "source TS: " << m_sourposvector.Ts << " "
         << "source Latitude: " << m_sourposvector.Lat << " "
         << "source Longitude: " << m_sourposvector.Lon << " "
         << "source Altitude: " << m_sourposvector.Alt << " "
         << "source Speed: " << m_sourposvector.Speed << " "
         << "source Heading: " << m_sourposvector.Heading << " "
         << "header type: " << (uint16_t) m_hType << " "
         << "TTL: " << (uint16_t) m_hLimit << " "
         << "payload length: " << m_pLength
        ;
}

void
GnCommonHeader::SetVersion (uint8_t version)
{
  m_version = version;
}

uint8_t
GnCommonHeader::GetVersion (void) const
{
  return m_version;
}

void
GnCommonHeader::SetNextHeader (uint8_t nexth)
{
  m_nHeader = nexth;
}

uint8_t
GnCommonHeader::GetNextHeader (void) const
{
  return m_nHeader;
}

void
GnCommonHeader::SetHtype (uint8_t type)
{
  m_hType = type;
}

uint8_t
GnCommonHeader::GetHtype (void) const
{
  return m_hType;
}

void
GnCommonHeader::SetHSubtype (uint8_t stype)
{
  m_hSubType = stype;
}

uint8_t
GnCommonHeader::GetHSubtype (void) const
{
  return m_hSubType;
}

void
GnCommonHeader::SetFlags (uint8_t flags)
{
 m_flags = flags;
}

uint8_t
GnCommonHeader::GetFlags (void) const
{
 return m_flags;
}

void
GnCommonHeader::SetLength (uint16_t plength)
{
 m_pLength = plength;
}

uint16_t
GnCommonHeader::GetLength (void) const
{
 return m_pLength;
}

void
GnCommonHeader::SetTrafClass (uint8_t tclass)
{
  m_tClass = tclass;
}

uint8_t
GnCommonHeader::GetTrafClass (void) const
{
  return m_tClass;
}

void
GnCommonHeader::SetHopLimit (uint8_t ttl)
{
 m_hLimit = ttl;
}

uint8_t
GnCommonHeader::GetHopLimit (void) const
{
 return m_hLimit;
}

void
GnCommonHeader::SetSourPosVector (struct LongPositionVector vector)
{
  m_sourposvector.gnAddr = vector.gnAddr;
  m_sourposvector.Ts = vector.Ts;
  m_sourposvector.Lat = vector.Lat;
  m_sourposvector.Lon = vector.Lon;
  m_sourposvector.Speed = vector.Speed;
  m_sourposvector.Heading = vector.Heading;
  m_sourposvector.Alt = vector.Alt;
  m_sourposvector.TimeAcc = vector.TimeAcc;
  m_sourposvector.PosAcc = vector.PosAcc;
  m_sourposvector.SpeedAcc = vector.SpeedAcc;
  m_sourposvector.HeadingAcc = vector.HeadingAcc;
  m_sourposvector.AltAcc = vector.AltAcc;
}

struct GnCommonHeader::LongPositionVector
GnCommonHeader::GetSourPosVector (void) const
{
  return m_sourposvector;
}

double CartesianDistance (double lat2, double lon2, double lat1, double lon1)
{
  double distance;
  distance = sqrt(((lat1-lat2)*(lat1-lat2))+((lon1-lon2)*(lon1-lon2)));
  return distance;
}

double ArcInRadians(uint32_t lat2, uint32_t lon2, uint32_t lat1, uint32_t lon1) {
  double latitudeArc  = (lat1 - lat2) * DEG_TO_RAD;
  double longitudeArc = (lon1 - lon2) * DEG_TO_RAD;
  double latitudeH = sin(latitudeArc * 0.5);
  latitudeH *= latitudeH;
  double lontitudeH = sin(longitudeArc * 0.5);
  lontitudeH *= lontitudeH;
  double tmp = cos(lat1*DEG_TO_RAD) * cos(lat2*DEG_TO_RAD);
  return 2.0 * asin(sqrt(latitudeH + tmp*lontitudeH));
}

double DistanceInMeters(uint32_t lat2, uint32_t lon2, uint32_t lat1, uint32_t lon1) {
  //std::cout<<"DistanceInMeters :  Distance = "<<EARTH_RADIUS_IN_METERS*ArcInRadians(lat2, lon2, lat1, lon1)<<std::endl;
  return EARTH_RADIUS_IN_METERS*ArcInRadians(lat2, lon2, lat1, lon1);
}

double bearingDegrees(double lat1, double lon1, double lat2, double lon2){
  double latRad1 = (lat1) * DEG_TO_RAD;
  double latRad2 = (lat2) * DEG_TO_RAD;
  double lngDiffRad = (lon2 - lon1) * DEG_TO_RAD;
  double y = sin(lngDiffRad) * cos(latRad2);
  double x = cos(latRad1) * sin(latRad2) - sin(latRad1) * cos(latRad2) * cos(lngDiffRad);

  return fmod((atan2(y, x) / DEG_TO_RAD) + 360, 360);
}

};
