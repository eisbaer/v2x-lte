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
#ifndef C2C_COMMON_HEADER_H
#define C2C_COMMON_HEADER_H

#include <math.h>

#include "ns3/header.h"
#include "ns3/nstime.h"

namespace ns3 {

double CartesianDistance (double lat2, double lon2, double lat1, double lon1);
double ArcInRadians(uint32_t lat2, uint32_t lon2, uint32_t lat1, uint32_t lon1);
double DistanceInMeters(uint32_t lat2, uint32_t lon2, uint32_t lat1, uint32_t lon1);

/**
 * @brief C2C Common network header
 *
 * This common network header implementation is based on the specification of the
 * version v0.0.5 of the draft ETSI/ITS-WG3 TS102 636-4-1
 *
 * It is based on the following format:
 *
 * 0               1               2               3
 * 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |Version|   NH  |  HT   |  HST  |    Reserved   |     Flags    |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                PL             |    TC         |       HL     |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                           GN_Addr                            |
 * |                                                              |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                               TST                            |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                Lat                           |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                                Long                          |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |             S                  |                H            |
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * |                  Alt           | TAcc |PosAcc | SAcc| HAcc|AAc|
 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *
 * @author: Fatma HRIZI <hrizi@eurecom.fr>
 * @Matthias ROECKL <matthias.roeckl@dlr.de>
 */
class GnCommonHeader : public Header
{
public:

  /**
  * Must be implemented to become a valid new header
  */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;

  struct LongPositionVector {
    uint64_t gnAddr;
    uint32_t Ts;
    uint32_t Lat;
    uint32_t Lon;
    uint16_t Speed; // _speedMetersPerSecond / 0.01
    uint16_t Heading; // _headingDegreesFromNorth / 0.1
    uint16_t Alt;
    uint8_t TimeAcc;
    uint8_t PosAcc;
    uint8_t SpeedAcc;
    uint8_t HeadingAcc;
    uint8_t AltAcc;

    double _speed;
    double _heading;
  };

  struct ShortPositionVector {
	uint64_t gnAddr;
	uint32_t Lat;
	uint32_t Long;
  };

  struct geoAreaPos
  {
	uint32_t lat;
	uint32_t lon;
  };

  /**
  * Allow protocol specific access to the header data
  */

  void SetVersion (uint8_t version);
  uint8_t GetVersion (void) const;

  void SetNextHeader (uint8_t nexth);
  uint8_t GetNextHeader (void) const;

  void SetHtype (uint8_t type);
  uint8_t GetHtype (void) const;

  void SetHSubtype (uint8_t stype);
  uint8_t GetHSubtype (void) const;

  void SetFlags (uint8_t flags);
  uint8_t GetFlags (void) const;

  void SetLength (uint16_t plength);
  uint16_t GetLength (void) const;

  void SetTrafClass (uint8_t tclass);
  uint8_t GetTrafClass (void) const;

  void SetHopLimit (uint8_t ttl);
  uint8_t GetHopLimit (void) const;

  void SetSourPosVector (struct LongPositionVector vector);
  struct LongPositionVector GetSourPosVector (void) const;

private:
  /**
   * @brief Version
   * 4-bit selector.
   * Identifies the version of the GeoNetworking protocol. The version field is set by the standard that defines the GeoNetworking media-dependent functionalities.
   */
  uint8_t m_version;

  /**
   * @brief NH (Next Header)
   * 4-bit unsigned integer.
   * Identifies the type of header immediately following the GeoNetworking header.
   *
   *   Type  |  Value  |    Description
   * --------|---------|----------------------
   *   ANY   |   0     |    Unspecified
   *   BTP   |   1     |    Transport protocol
   *   IPv6  |   2     |    IPv6 header
   */
  uint8_t m_nHeader;

  /**
   * @brief HT (Header Type)
   * 4-bit selector.
   * Identifies the type of GeoNetworking Header.
   */
  uint8_t m_hType;

  /**
   * @brief HST (Header Subtype)
   * 4-bit selector.
   * Identifies the subtype of the given type of GeoNetworking Header
   */
  uint8_t m_hSubType;

  /**
   * @brief Reserved
   * 8 bits.
   * Reserved for media-dependent functionality.
   */
  uint8_t m_reserved;

  /**
   * @brief Flags
   * 8 bits: XXXXXXRX
   * R: 0 Vehicle ITS station, 1 Roadside ITS station.
   * X: reserved for GeoNetworking media-dependent functionalities.
   * Flags to distinguish ITS stations.
   */
  uint8_t m_flags;

  /**
    * @brief PL (Payload Length)
    * 16-bit unsigned integer.
    * Length of the Network Header payload, i.e., the rest of the packet following the whole GeoNetworking header in octets.
    */
  uint16_t m_pLength;

  /**
    * @brief TC (Traffic Class)
    * 8-bit unsigned integer.
    * The traffic class shall be represented by three sub-fields, i.e., relevance, reliability and latency.
    */
  uint8_t m_tClass;

  /**
   * @brief Hop Limit
   * 8-bit unsigned integer.
   * Decremented by 1 by each GeoAdhoc router that forwards the packet. The packet must not be forwarded if Hop Limit is decremented to zero.
   */
  uint8_t m_hLimit;


  struct LongPositionVector m_sourposvector;
};

};//namespace ns3

#endif   /*C2C_COMMON_HEADER_H*/
