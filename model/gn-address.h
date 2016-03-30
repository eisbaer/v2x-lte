/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009-2010, EURECOM, EU FP7 iTETRIS project
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
 * Author: Fatma Hrizi <fatma.hrizi@eurecom.fr>
 */

#ifndef C2C_ADDRESS_H
#define C2C_ADDRESS_H

#include "ns3/attribute-helper.h"
#include "ns3/object.h"
#include "gn-common-header.h"

namespace ns3 {

/**
 * \ingroup address
 *
 * \brief c2c addresses are stored in this class.
 */
class GnAddress : public Object{
public:

  static TypeId GetTypeId (void);
  GnAddress ();
  ~GnAddress ();

  static const uint32_t BROAD;
  static const uint32_t ANYCAST;

  void Set (GnAddress addr);
  void Set (uint64_t id, struct GnCommonHeader::geoAreaPos* areapos1, struct GnCommonHeader::geoAreaPos* areapos2, uint32_t size);
  void Set (uint64_t id, uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2, uint32_t size);
  void Set (uint64_t id, struct GnCommonHeader::geoAreaPos* areapos1);
  void Set (uint64_t id, uint32_t lat1, uint32_t lon1);
  void Set (uint64_t id, uint32_t ttl);
  uint64_t GetId (void) const;
  void SetId (uint64_t id);

  uint32_t GetAreaSize (void) const;
  void SetAreaSize (uint32_t size);

  void SetGeoAreaPos1 (struct GnCommonHeader::geoAreaPos* areapos1);
  struct GnCommonHeader::geoAreaPos*  GetGeoAreaPos1 (void) const;

  void SetGeoAreaPos2 (struct GnCommonHeader::geoAreaPos* areapos2);
  struct GnCommonHeader::geoAreaPos*  GetGeoAreaPos2 (void) const;

  /**
   * \brief Print this address to the given output stream
   *
   * \param os The output stream to which this C2C Address is printed
   */
  void Print (std::ostream &os) const;

  /**
    * \return true if address is for Geo broadcast; false otherwise
    */
  bool IsGeoBroadcast (void) const;

  /**
    * \return true if address is for Geo Unicast; false otherwise
    */
  bool IsGeoUnicast (void) const;

  /**
    * \return true if address is for Geo Anycast; false otherwise
    */
  bool IsGeoAnycast (void) const;

  /**
    * \return true if address is for Topo broadcast; false otherwise
    */
  bool IsTopoBroadcast (void) const;

  /**
    * \return true if address is for broadcast; false otherwise
    */
  bool IsBroadcast (void) const;


private:
  uint64_t m_id;

  struct GnCommonHeader::geoAreaPos* m_geoareapos1;
  struct GnCommonHeader::geoAreaPos* m_geoareapos2;
  uint32_t m_areasize;
  friend bool operator == (GnAddress const &a, GnAddress const &b);
  friend bool operator != (GnAddress const &a, GnAddress const &b);
};

std::ostream& operator<< (std::ostream& os, GnAddress const& address);
std::istream & operator >> (std::istream &is, GnAddress &address);

inline bool operator == (const GnAddress &a, const GnAddress &b)
{
  return ((a.m_id == b.m_id)&&(a.m_geoareapos1->lat == b.m_geoareapos1->lat)&&(a.m_geoareapos1->lon == b.m_geoareapos1->lon)&&(a.m_geoareapos2->lat == b.m_geoareapos2->lat)&&(a.m_geoareapos2->lon == b.m_geoareapos2->lon)&&(a.m_areasize == b.m_areasize));
}
inline bool operator != (const GnAddress &a, const GnAddress &b)
{
  return (((a.m_id != b.m_id)&&(a.m_geoareapos1->lat != b.m_geoareapos1->lat)&&(a.m_geoareapos1->lon != b.m_geoareapos1->lon)&&(a.m_geoareapos2->lat != b.m_geoareapos2->lat)&&(a.m_geoareapos2->lon != b.m_geoareapos2->lon)&&(a.m_areasize != b.m_areasize)));
}

} // namespace ns3

#endif /* C2C_ADDRESS_H */
