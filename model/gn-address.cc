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

#include <stdlib.h>

#include "gn-address.h"
#include "ns3/log.h"
#include "ns3/assert.h"

NS_LOG_COMPONENT_DEFINE ("gnAddress");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (GnAddress);

const uint32_t GnAddress::ANYCAST = 0x9998;
const uint32_t GnAddress::BROAD = 0x9999;

TypeId
GnAddress::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GnAddress")
    .SetParent<Object> ()
    .AddConstructor<GnAddress> ()
    ;
  return tid;
}

GnAddress::GnAddress ()
{}

GnAddress::~GnAddress ()
{
  free (m_geoareapos1);
  if (m_geoareapos2 != NULL)
  free (m_geoareapos2);
}

//Geo destination address
void
GnAddress::Set (uint64_t id, struct GnCommonHeader::geoAreaPos* areapos1, struct GnCommonHeader::geoAreaPos* areapos2, uint32_t size)
{
  m_geoareapos1 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos2 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_id = id;
  m_geoareapos1->lat = areapos1->lat;
  m_geoareapos1->lon = areapos1->lon;
  m_geoareapos2->lat = areapos2->lat;
  m_geoareapos2->lon = areapos2->lon;
  m_areasize = size;
}

void
GnAddress::Set (uint64_t id, uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2, uint32_t size)
{
  m_id = id;
  m_geoareapos1 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos2 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos1->lat = lat1;
  m_geoareapos1->lon = lon1;
  m_geoareapos2->lat = lat2;
  m_geoareapos2->lon = lon2;
  m_areasize = size;
}

//Node address
void
GnAddress::Set (uint64_t id, struct GnCommonHeader::geoAreaPos* areapos1)
{
  m_id = id;
  m_geoareapos1 = areapos1;
  m_geoareapos2 = NULL;
}

//TOPO address
void
GnAddress::Set (uint64_t id, uint32_t ttl)
{
  m_id = id;
  m_geoareapos1 = NULL;
  m_geoareapos2 = NULL;
  m_areasize = ttl;
}

void
GnAddress::Set (uint64_t id, uint32_t lat1, uint32_t lon1)
{
  m_id = id;
  m_geoareapos1 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos1->lat = lat1;
  m_geoareapos1->lon = lon1;
  m_geoareapos2 = NULL;
}

void
GnAddress::Set (GnAddress addr)
{
  m_id = addr.GetId ();
  m_geoareapos1 = addr.GetGeoAreaPos1 ();
  m_geoareapos2 = addr.GetGeoAreaPos2 ();
  m_areasize = addr.GetAreaSize ();
}

void
GnAddress::Print (std::ostream &os) const
{
}

uint64_t
GnAddress::GetId (void) const
{
  return m_id;
}

void
GnAddress::SetId (uint64_t id)
{
  m_id = id;
}

uint32_t
GnAddress::GetAreaSize (void) const
{
  return m_areasize;
}

void
GnAddress::SetAreaSize (uint32_t size)
{
  m_areasize = size;
}

void
GnAddress::SetGeoAreaPos1 (struct GnCommonHeader::geoAreaPos*  areapos1)
{
  m_geoareapos1 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos1->lat = areapos1->lat;
  m_geoareapos1->lon = areapos1->lon;
}

struct GnCommonHeader::geoAreaPos*
GnAddress::GetGeoAreaPos1 (void) const
{
  return m_geoareapos1;
}

void
GnAddress::SetGeoAreaPos2 (struct GnCommonHeader::geoAreaPos*  areapos2)
{
  m_geoareapos2 = (struct GnCommonHeader::geoAreaPos*) malloc (sizeof (struct GnCommonHeader::geoAreaPos));
  m_geoareapos2->lat = areapos2->lat;
  m_geoareapos2->lon = areapos2->lon;
}

struct GnCommonHeader::geoAreaPos*
GnAddress::GetGeoAreaPos2 (void) const
{
  return m_geoareapos2;
}

bool
GnAddress::IsGeoBroadcast (void) const
{
  return (m_id == BROAD && m_geoareapos1 != NULL);
}

bool
GnAddress::IsBroadcast (void) const
{
  return (m_id == BROAD);
}

bool
GnAddress::IsGeoUnicast (void) const
{
  return (m_id != BROAD && m_id != ANYCAST && m_geoareapos2 == NULL);
}

bool
GnAddress::IsGeoAnycast (void) const
{
  return (m_id == ANYCAST);
}

bool
GnAddress::IsTopoBroadcast (void) const
{
  return (m_id == BROAD && m_geoareapos1 == NULL);
}

} // namespace ns3
