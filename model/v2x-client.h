/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef V2X_CLIENT_H
#define V2X_CLIENT_H

#include "ns3/application.h"
#include "ns3/mobility-module.h"

#include "gn-common-header.h"
#include "location-table.h"


namespace ns3 {

  int32_t timeToTaiMilliSecondsSince2004Mod32(Time timeX);
  Time milliSecondsMod32ToTime(int32_t intMilliSecondsX);

  class V2xClient : public Application
  {
  public:

    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId (void);

    V2xClient ();

    virtual ~V2xClient ();

  protected:
    virtual void DoDispose (void);

  private:

    virtual void StartApplication (void);
    virtual void StopApplication (void);

    void ConnectionSucceeded (Ptr<Socket> socket);
    void ConnectionFailed (Ptr<Socket> socket);
    /**
     * \brief Send a packet
     */
    void SendPacket (Ptr<Packet> packet);
    void SendCAM (bool withLowFreq);
    void checkTriggeringConditions (void);

    void StartListeningLocal (void);
    void StopListeningLocal (void);

    void Receive (Ptr<Socket> socket);

    void AquireMobilityInfo (void);
    bool farEnough(GnCommonHeader::LongPositionVector oldPos, GnCommonHeader::LongPositionVector newPos);

    Address m_peerAddress;
    Address m_peerListening;

    EventId m_checkEvent; //!< Event to triggering check

    Ptr<Socket> m_send_socket; //!< Socket
    EventId m_sendEvent; //!< Event to send the next packet
    uint32_t m_sentCounter; //!< Counter for sent packets

    Ptr<Socket> m_socket; //!< Socket

    // AquireMobilityInfo
    GnCommonHeader::LongPositionVector m_currentSourPosVector;
    Ptr<MobilityModel> m_mobilityModel;

    // SendCAM
    Time m_interval;

    bool m_withCam;
    Time m_cam_lastSend;
    Time m_cam_lastSendLowFreq;
    GnCommonHeader::LongPositionVector m_cam_lastSendSourPosVector;
    uint16_t m_cam_highDynamics;

  };

}

#endif /* V2X_CLIENT_H */

