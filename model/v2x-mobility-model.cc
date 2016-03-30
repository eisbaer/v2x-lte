/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 University of Athens (UOA)
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
 * Author:  - Lampros Katsikas <lkatsikas@di.uoa.gr>
 *          - Konstantinos Chatzikokolakis <kchatzi@di.uoa.gr>
 */

#include "v2x-mobility-model.h"

#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <math.h>

NS_LOG_COMPONENT_DEFINE ("V2xMobilityModel");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (V2xMobilityModel);

TypeId
V2xMobilityModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::V2xMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Mobility")
    .AddConstructor<V2xMobilityModel> ()
    .AddAttribute ("Bounds",
                   "Bounds of the area to cruise.",
                   RectangleValue (Rectangle (0.0, 0.0, 100.0, 100.0)),
                   MakeRectangleAccessor (&V2xMobilityModel::m_bounds),
                   MakeRectangleChecker ())
    .AddAttribute ("Time",
                   "Change current direction and speed after moving for this delay.",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&V2xMobilityModel::m_modeTime),
                   MakeTimeChecker ())
    .AddAttribute ("Distance",
                   "Change current direction and speed after moving for this distance.",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&V2xMobilityModel::m_modeDistance),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Mode",
                   "The mode indicates the condition used to "
                   "change the current speed and direction",
                   EnumValue (V2xMobilityModel::MODE_DISTANCE),
                   MakeEnumAccessor (&V2xMobilityModel::m_mode),
                   MakeEnumChecker (V2xMobilityModel::MODE_DISTANCE, "Distance",
                                    V2xMobilityModel::MODE_TIME, "Time"))
    .AddAttribute ("Direction",
                   "A random variable used to pick the direction (gradients).",
                   StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                   MakePointerAccessor (&V2xMobilityModel::m_directionVariable),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("Speed",
                   "A random variable used to pick the speed (m/s).",
                   StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),
                   MakePointerAccessor (&V2xMobilityModel::m_speed),
                   MakePointerChecker<RandomVariableStream> ())
    .AddAttribute ("SpeedVariation",
                    "A random variable used to pick the speed variation (m/s).",
                    DoubleValue (1.0),
                    MakeDoubleAccessor (&V2xMobilityModel::m_speedVariation),
                    MakeDoubleChecker<double> ());
  return tid;
}


void V2xMobilityModel::SetDirection(const Vector& direction){
	//m_direction
}

Vector V2xMobilityModel::GetDirection(void) const{
	Vector vector (std::cos (m_direction), std::sin (m_direction), 0.0);

	return vector;
}

void
V2xMobilityModel::SetSpeedVariation(double variation){
    m_speedVariation = variation;
}

void
V2xMobilityModel::DoInitialize (void)
{
  DoInitializePrivate ();
  MobilityModel::DoInitialize ();
}

void
V2xMobilityModel::DoInitializePrivate (void)
{
  m_helper.Update ();
  double speed = m_speed->GetValue ();
  m_direction = 1.0;

  Ptr<UniformRandomVariable> randomVariable = CreateObject<UniformRandomVariable> ();
  double s = ceil(randomVariable->GetValue(-m_speedVariation, m_speedVariation) * 100 + 0.5)/100;

  Vector vector (m_direction * (speed+s),
                 0 ,
                 0.0);
  m_helper.SetVelocity (vector);
  m_helper.Unpause ();

  Time delayLeft;
  if (m_mode == V2xMobilityModel::MODE_TIME)
    {
      delayLeft = m_modeTime;
    }
  else
    {
      delayLeft = Seconds (m_modeDistance / speed);
    }
  DoWalk (delayLeft);
}

void
V2xMobilityModel::DoWalk (Time delayLeft)
{
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  Vector nextPosition = position;
  nextPosition.x += speed.x * delayLeft.GetSeconds ();
  nextPosition.y += speed.y * delayLeft.GetSeconds ();
  m_event.Cancel ();
  if (m_bounds.IsInside (nextPosition))
    {
      m_event = Simulator::Schedule (delayLeft, &V2xMobilityModel::DoInitializePrivate, this);
    }
  else
    {
      nextPosition = m_bounds.CalculateIntersection (position, speed);
      Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
      m_event = Simulator::Schedule (delay, &V2xMobilityModel::Rebound, this,
                                     delayLeft - delay);
    }
  NotifyCourseChange ();
}

void
V2xMobilityModel::Rebound (Time delayLeft)
{
  m_helper.UpdateWithBounds (m_bounds);
  Vector position = m_helper.GetCurrentPosition ();
  Vector speed = m_helper.GetVelocity ();
  switch (m_bounds.GetClosestSide (position))
    {
    case Rectangle::RIGHT:
    case Rectangle::LEFT:
      speed.x = -speed.x;
      break;
    case Rectangle::TOP:
    case Rectangle::BOTTOM:
      speed.y = -speed.y;
      break;
    }
  m_helper.SetVelocity (speed);
  m_helper.Unpause ();
  DoWalk (delayLeft);
}

void
V2xMobilityModel::DoDispose (void)
{
  // chain up
  MobilityModel::DoDispose ();
}
Vector
V2xMobilityModel::DoGetPosition (void) const
{
  m_helper.UpdateWithBounds (m_bounds);
  return m_helper.GetCurrentPosition ();
}
void
V2xMobilityModel::DoSetPosition (const Vector &position)
{
  NS_ASSERT (m_bounds.IsInside (position));
  m_helper.SetPosition (position);
  Simulator::Remove (m_event);
  m_event = Simulator::ScheduleNow (&V2xMobilityModel::DoInitializePrivate, this);
}
Vector
V2xMobilityModel::DoGetVelocity (void) const
{
  return m_helper.GetVelocity ();
}
int64_t
V2xMobilityModel::DoAssignStreams (int64_t stream)
{
  m_speed->SetStream (stream);
  m_directionVariable->SetStream (stream + 1);
  return 2;
}


} // namespace ns3
