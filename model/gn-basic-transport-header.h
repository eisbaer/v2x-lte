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

#ifndef C2C_TRANSPORT_HEADER_H
#define C2C_TRANSPORT_HEADER_H

#include "ns3/header.h"

namespace ns3 {

/**
 * \ingroup c2cTransport
 * \brief iTETRIS [WP600] - Packet header for c2cTransport packets
 *
 * This class has fields corresponding to those in a network c2c Transport header
 * (port numbers, payload size, checksum) as well as methods for serialization
 * to and deserialization from a byte buffer.
 */
class GnBasicTransportHeader : public Header
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

  //Setters
    /**
     * \param port The source port for this TcpHeader
     */
    void SetSourcePort (uint16_t port);
    /**
     * \param port the destination port for this TcpHeader
     */
    void SetDestinationPort (uint16_t port);

    //Getters
      /**
       * \param port The source port for this TcpHeader
       */
    uint16_t GetSourcePort (void) const;
      /**
       * \param port the destination port for this TcpHeader
       */
    uint16_t GetDestinationPort (void) const;

private:

    uint16_t m_sourcePort;
    uint16_t m_destinationPort;

};

} // namespace ns3

#endif /* C2C_TRANSPORT_HEADER */
