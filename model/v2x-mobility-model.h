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

#ifndef V2X_MOBILITY_MODEL_H
#define V2X_MOBILITY_MODEL_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/rectangle.h"
#include "ns3/random-variable-stream.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-velocity-helper.h"

namespace ns3 {


/**
 * \ingroup mobility
 * \brief Constant direction, random velocity mobility model
 *
 */
class V2xMobilityModel : public MobilityModel
{
public:
  static TypeId GetTypeId (void);

  enum Mode  {
    MODE_DISTANCE,
    MODE_TIME
  };

  /**
   * \param direction the direction of the node
   */
  void SetDirection (const Vector& direction);

  /**
   * \return the direction of the node
   */
  Vector GetDirection (void) const;

  /**
   * @brief SetSpeedVariation
   * @param variation
   */
  void SetSpeedVariation (double variation);

private:
  void Rebound (Time timeLeft);
  void DoWalk (Time timeLeft);
  void DoInitializePrivate (void);
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  virtual int64_t DoAssignStreams (int64_t);

  ConstantVelocityHelper m_helper;
  EventId m_event;
  enum Mode m_mode;
  double m_modeDistance;
  Time m_modeTime;
  Ptr<RandomVariableStream> m_speed;
  double m_direction;
  Ptr<RandomVariableStream> m_directionVariable;
  Rectangle m_bounds;

  double m_speedVariation;

};


} // namespace ns3

#endif /* V2X_MOBILITY_MODEL_H */
