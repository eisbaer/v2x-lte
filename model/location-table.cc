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

//#include "beaconing-protocol.h"
#include "ns3/simulator.h"
#include "ns3/mobility-model.h"

#include "location-table.h"

NS_LOG_COMPONENT_DEFINE ("LocationTable");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (LocationTable);
const float LocationTable::LOCATION_EGO_INTERVAL = 0.01;
const float LocationTable::LOCATION_ENTRY_LIFETIME = 5.0;

TypeId
LocationTable::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LocationTable")
    .SetParent<Object> ()
    .AddConstructor<LocationTable> ()
    ;
  return tid;
}

LocationTable::LocationTable ()
{
  NS_LOG_FUNCTION (this);
  Simulator::ScheduleNow (&LocationTable::ScheduleUpdatePos, this); // schedule update own position
  Simulator::ScheduleNow (&LocationTable::ScheduleCleanTable, this); //schedule clean table
}

LocationTable::~LocationTable ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_updateEvent);
  Simulator::Cancel (m_posEvent);
}

void
LocationTable::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_node = 0;
  Object::DoDispose ();
}

void
LocationTable::SetNode (Ptr<Node> node)
{
  NS_LOG_FUNCTION (this);
  m_node = node;
}

void
LocationTable::NotifyNewAggregate ()
{
  if (m_node == 0)
    {
      Ptr<Node>node = this->GetObject<Node>();
      // verify that it's a valid node and that
      // the node has not been set before
      if (node != 0)
        {
          this->SetNode (node);
        }
    }
  Object::NotifyNewAggregate ();
}

void
LocationTable::ScheduleUpdatePos ()
{
  m_posEvent = Simulator::ScheduleNow (&LocationTable::AddOwnEntry, this);
}

void
LocationTable::ScheduleCleanTable ()
{
  m_updateEvent = Simulator::ScheduleNow (&LocationTable::CleanTable, this);
}

void
LocationTable::AddOwnEntry ()
{
  struct GnCommonHeader::LongPositionVector vector;
  if (m_node != 0)
  {
  vector.gnAddr = m_node->GetId ();
  vector.Ts = (uint32_t) ((Simulator::Now ()).GetSeconds ());

  // Retrieving position information (latitude, longitude, altitude), heading and speed from the mobility model

  Ptr<MobilityModel> model = m_node->GetObject<MobilityModel> ();
  if (model != 0)
    {
    vector.Lat = (uint32_t) model->GetPosition ().x;
    vector.Lon = (uint32_t) model->GetPosition ().y;
    vector.Speed = (uint32_t) model->GetVelocity ().x;
  //   vector.Alt = /*Altitude*/;
  //   vector.Heading = /*Heading*/;
  //   vector.PosAcc = /*Position Accuracy*/;
  //   vector.AltAcc = /*Altitude Accuracy*/;
  //   vector.SpeedAcc = /*Speed accuracy*/;
  //   vector.HeadingAcc = /*Heading Accuracy*/;
    }
  }

  Ptr <LocationTable> ntable = m_node->GetObject<LocationTable> ();
  NS_LOG_INFO  ("LOCATION_TABLE on node " << m_node->GetId() << " add own entry called");
  ntable->AddPosEntry (vector);

  m_posEvent = Simulator::Schedule (Seconds (LocationTable::LOCATION_EGO_INTERVAL), &LocationTable::AddOwnEntry, this);
}

void
LocationTable::AddPosEntry (GnCommonHeader commonheader)
{
  bool exist = false;
  struct LocTableEntry entry;
  entry.gnAddr = commonheader.GetSourPosVector ().gnAddr;
  entry.Ts = commonheader.GetSourPosVector ().Ts;
  entry.Lat = commonheader.GetSourPosVector ().Lat;
  entry.Lon = commonheader.GetSourPosVector ().Lon;
  entry.Alt = commonheader.GetSourPosVector ().Alt;
  entry.PosAcc = commonheader.GetSourPosVector ().PosAcc;
  entry.AltAcc = commonheader.GetSourPosVector ().AltAcc;
  entry.Speed = commonheader.GetSourPosVector ().Speed;
  entry.Heading = commonheader.GetSourPosVector ().Heading;
  entry.SpeedAcc = commonheader.GetSourPosVector ().SpeedAcc;
  entry.HeadingAcc = commonheader.GetSourPosVector ().HeadingAcc;

//   if ((uint16_t)commonheader.GetHtype() == Node::C2C_BEACON)
   entry.is_neigh = true;
//   else
//    entry.is_neigh = false;

  if (m_Table.size() != 0)
  {
  for (Table::iterator i = m_Table.begin ();
       i != m_Table.end (); i++)
    {

    if (i->gnAddr == entry.gnAddr)
    {
     exist = true;
     if (i->Ts == entry.Ts)
      return;
     else
      {
       m_Table.erase (i);
       m_Table.push_back (entry);
       return;
      }
     }

    }
  if (exist == false)
    m_Table.push_back (entry);
  }
  else
    m_Table.push_back (entry);
}

void
LocationTable::AddPosEntry (struct GnCommonHeader::LongPositionVector vector)
{
  bool exist = false;
  struct LocTableEntry entry;
  entry.gnAddr = vector.gnAddr;
  entry.Ts = vector.Ts;
  entry.Lat = vector.Lat;
  entry.Lon = vector.Lon;
  entry.Alt = vector.Alt;
  entry.PosAcc = vector.PosAcc;
  entry.AltAcc = vector.AltAcc;
  entry.Speed = vector.Speed;
  entry.Heading = vector.Heading;
  entry.SpeedAcc = vector.SpeedAcc;
  entry.HeadingAcc = vector.HeadingAcc;
  entry.is_neigh = false;

  if (m_Table.size() != 0)
  {
    for (Table::iterator i = m_Table.begin (); i != m_Table.end (); i++)
    {

      if (i->gnAddr == vector.gnAddr)
      {
       NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " is in the table");
       exist = true;
       if (i->Ts == vector.Ts){
           NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " is in the table and I last stored it in this TS. I do not store it again      isneigh= "<<i->is_neigh);
           return;
           }
       else
       {
        bool was_neigh= i->is_neigh;
        m_Table.erase (i);
        if (was_neigh == true){

          entry.is_neigh = true;
          NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was a neigh so I store it as such          isneigh= "<< entry.is_neigh);
          }
        else{
          NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was NOT a neigh so I store it as such              isneigh= "<<entry.is_neigh);
         }
          m_Table.push_back (entry);
        return;
       }
      }
    }
    if (exist == false){
        if(vector.gnAddr == m_node->GetId()){
        entry.is_neigh = true;
           NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was not in the table. It is the local node. I store it as a neigh             isneigh= "<<entry.is_neigh);
        }
        else{
        NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was not in the table, I store it as not a neigh             isneigh= "<<entry.is_neigh);
        }
        m_Table.push_back (entry);
    }
  }
  else{
     if(vector.gnAddr == m_node->GetId()){
        entry.is_neigh = true;
           NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was not in the table. yhe table was empty. It is the local node. I store it as a neigh             isneigh= "<<entry.is_neigh);
        }
        else{
           NS_LOG_DEBUG  ("LOCATION_TABLE: Node "<< vector.gnAddr << " was not in the table (table was empty), I store it as not a neigh              isneigh= "<<entry.is_neigh);
    }
    m_Table.push_back (entry);
    }
}

void
LocationTable::CleanTable ()
{
  Time time = Simulator::Now ();
  double interval;
  if (m_Table.size() != 0)
  {
  for (Table::iterator i = m_Table.begin ();
       i != m_Table.end (); i++)
    {
     interval = (time.GetSeconds ()) - (i->Ts);
     if (interval >= LOCATION_ENTRY_LIFETIME)
     {
       m_Table.erase (i);
     }
    }
   }

  m_updateEvent = Simulator::Schedule (Seconds (LocationTable::LOCATION_ENTRY_LIFETIME), &LocationTable::CleanTable, this);
}

int
LocationTable::GetNbNeighs ()
{
  int nb_neigh = 0;
  if (m_Table.size() != 0)
  {
    for (Table::iterator i = m_Table.begin ();
       i != m_Table.end (); i++)
    {
     if (i->is_neigh == true)
     {
      nb_neigh++;
     }
    }
  }
  return nb_neigh-1;
}

LocationTable::Table
LocationTable::GetTable ()
{
  return m_Table;
}

} //namespace ns3
