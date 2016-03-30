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


#include "gn-basic-transport-header.h"

namespace ns3 {

TypeId
GnBasicTransportHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GnTransportHeader")
    .SetParent<Header> ()
    .AddConstructor<GnBasicTransportHeader> ()
  ;
  return tid;
}

TypeId
GnBasicTransportHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
GnBasicTransportHeader::GetSerializedSize (void) const
{
  /**
  * This is the size of a C2C Transport Header
  */

  return 4;
}

void
GnBasicTransportHeader::Serialize (Buffer::Iterator start) const
{

  Buffer::Iterator i = start;

   i.WriteHtonU16(m_sourcePort);
   i.WriteHtonU16(m_destinationPort);
}

uint32_t
GnBasicTransportHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;

  m_sourcePort = i.ReadNtohU16 ();
  m_destinationPort = i.ReadNtohU16 ();

  return GetSerializedSize(); // The number of bytes consumed.
}

void
GnBasicTransportHeader::Print (std::ostream &os) const
{
 os << "\n****GN Basic Transport Header**** "
         << "source Port: " << m_sourcePort << " "
         << "Destination Port: " << m_destinationPort
        ;
}

void
GnBasicTransportHeader::SetSourcePort (uint16_t port)
{
  m_sourcePort = port;
}
void
GnBasicTransportHeader::SetDestinationPort (uint16_t port)
{
  m_destinationPort = port;
}

uint16_t
GnBasicTransportHeader::GetSourcePort (void) const
{
	return m_sourcePort;
}
  /**
   * \param port the destination port for this TcpHeader
   */
uint16_t
GnBasicTransportHeader::GetDestinationPort (void) const
{
	return m_destinationPort;
}

}; // namespace ns3
