/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/socket.h"
#include "ns3/string.h"
#include "ns3/packet.h"
#include "ns3/address.h"
#include "ns3/pointer.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket.h"
#include "ns3/address-utils.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/packet-socket-address.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/lte-module.h"

#include "gn-address.h"
#include "gn-basic-transport-header.h"
#include "gn-common-header.h"
//#include "location-table.h"

#include "v2x-client.h"

#include "math.h"

//#include "gen/cam.h"


NS_LOG_COMPONENT_DEFINE ("V2xClient");

namespace ns3 {

  static int32_t LEAP_SECONDS_SINCE_2004 = 4; // Let's assume we're always in 2015.

  int32_t
  timeToTaiMilliSecondsSince2004Mod32(Time instantX) {

    int32_t millis2004 = Years(12).GetMilliSeconds();
    int32_t millisAtX = Simulator::Now().GetMilliSeconds();
    int32_t taiMillis = (millisAtX + LEAP_SECONDS_SINCE_2004*1000) - millis2004;

    return taiMillis % (1L << 32);
  }

  // Returns the nearest to now that will have given amount of TAI millis since 2004.
  Time
  milliSecondsMod32ToTime(int32_t intMilliSecondsX) {
    long millisX = (long)(intMilliSecondsX);  // unsigned int...
    Time now = Simulator::Now();
    long millisNow = timeToTaiMilliSecondsSince2004Mod32(now);
    long delta = millisNow - millisX;
    // Small positive delta is what we expect.
    // Small negative delta is fine too, it would mean that the packet came from the future,
    // which can be explained by our clock being a little behind.
    // Huge negative delta is from previous mod32, and should be changed to small positive.
    // Huge positive delta might come from a packet a little from the future and next mod32,
    // we want instead a small negative delta.
    if (delta < -(1L << 31)) { delta += (1L << 32); }
    if (delta >  (1L << 31)) { delta -= (1L << 32); }
    Time instantX = now - MilliSeconds(delta);
    return instantX;
  }

// its
//static double MICRODEGREE = 1E-6;
//static uint8_t REPLY_SUCCESS = 0x01;
//static uint8_t REPLY_FAILURE = 0x00;

static uint16_t PORT_CAM  = 2001;
// static uint16_t PORT_DENM = 2002;
// static uint16_t PORT_MAP  = 2003;
// static uint16_t PORT_SPAT = 2004;

static uint32_t CHECK_CAM_GEN = 10;
static int64_t CAM_INTERVAL_MIN_MS = 100;
static int64_t CAM_INTERVAL_MAX_MS = 1000;
static int64_t CAM_LOW_FREQ_INTERVAL_MS = 500;

static short HIGH_DYNAMICS_CAM_COUNT = 4;

//static double CAM_LIFETIME_SECONDS = 0.9;


NS_OBJECT_ENSURE_REGISTERED (V2xClient);

TypeId
V2xClient::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::V2xClient")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<V2xClient> ()
    .AddAttribute ("RemoteAddress",
                   "The destination Address of the outbound packets",
                   AddressValue (),
                   MakeAddressAccessor (&V2xClient::m_peerAddress),
                   MakeAddressChecker ())
    .AddAttribute ("WithCAM",
		   "Activate CAMs",
		   BooleanValue(true),
		   MakeBooleanAccessor (&V2xClient::m_withCam),
		   MakeBooleanChecker ())

  ;
  return tid;
}

V2xClient::V2xClient ()
{
  NS_LOG_FUNCTION (this);

  m_send_socket = 0;

  m_sentCounter = 0;

  m_interval = MilliSeconds (CAM_INTERVAL_MIN_MS);
  m_sendEvent = EventId ();

  m_cam_highDynamics = 0;
}

V2xClient::~V2xClient ()
{
  NS_LOG_FUNCTION (this);

  m_send_socket = 0;

  m_sentCounter = 0;

  m_interval = MilliSeconds (CAM_INTERVAL_MIN_MS);
  m_sendEvent = EventId ();
}

void
V2xClient::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  m_send_socket = 0;

  Application::DoDispose ();
}

void
V2xClient::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  TypeId m_tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

  // Create the socket if not already
  if (!m_send_socket)
    {
      m_send_socket = Socket::CreateSocket (GetNode (), m_tid);

      m_send_socket->Bind ();

      m_send_socket->Connect (InetSocketAddress (Ipv4Address::ConvertFrom(m_peerAddress), PORT_CAM));

      m_send_socket->SetAllowBroadcast (true);
      m_send_socket->ShutdownRecv ();

      m_send_socket->SetConnectCallback (
	MakeCallback (&V2xClient::ConnectionSucceeded, this),
        MakeCallback (&V2xClient::ConnectionFailed, this)
	);
     }

  StartListeningLocal();

  if(m_withCam)
    {
      m_checkEvent = Simulator::Schedule(MilliSeconds(CHECK_CAM_GEN), &V2xClient::checkTriggeringConditions, this);
    }

  AquireMobilityInfo();

}

void
V2xClient::StartListeningLocal (void)    // Called at time specified by Start
{
    NS_LOG_FUNCTION (this);

    TypeId m_tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	// Create the socket if not already
	if (!m_socket) {
	    m_socket = Socket::CreateSocket(GetNode(), m_tid);
	    m_socket->Bind(InetSocketAddress (Ipv4Address::GetAny(), PORT_CAM));
	    m_socket->Listen();
	    m_socket->ShutdownSend();
//		if (addressUtils::IsMulticast(m_peerListening)) {
//			Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket>(m_socketListening);
//			if (udpSocket) {
//				// equivalent to setsockopt (MCAST_JOIN_GROUP)
//				udpSocket->MulticastJoinGroup(0, m_peerListening);
//			} else {
//				NS_FATAL_ERROR("Error: joining multicast on a non-UDP socket");
//			}
//		}
	}

	m_socket->SetRecvCallback(MakeCallback(&V2xClient::Receive, this));
//	m_socketListening->SetAcceptCallback(
//			MakeNullCallback<bool, Ptr<Socket>, const Address &>(),
//            MakeCallback(&V2vControlClient::HandleAccept, this));
//	m_socketListening->SetCloseCallbacks(
//            MakeCallback(&V2vControlClient::HandlePeerClose, this),
//            MakeCallback(&V2vControlClient::HandlePeerError, this));
}

void
V2xClient::StopApplication (void)
{
  NS_LOG_FUNCTION (this);

  if (m_send_socket != 0) {
      m_send_socket->Close();
      m_send_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket> > ());
      m_send_socket = 0;
  } else {
      NS_LOG_WARN("V2xClient found null socket to close in StopApplication");
  }
  Simulator::Cancel (m_checkEvent);
  Simulator::Cancel (m_sendEvent);

  StopListeningLocal();
}

void
V2xClient::StopListeningLocal (void)     // Called at time specified by Stop
{
  NS_LOG_FUNCTION (this);
  if (m_socket != 0)
    {
      m_socket->Close ();
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      m_socket = 0;
    }
}

void
V2xClient::ConnectionSucceeded (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}

void
V2xClient::ConnectionFailed (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}

void
V2xClient::AquireMobilityInfo (void)
{
  NS_LOG_FUNCTION (this);

  // mobility model
  Ptr<MobilityModel> model = GetNode ()->GetObject<MobilityModel> ();

  if (model == 0)
    {
      NS_FATAL_ERROR("V2xClient needs a MobilityModel on node " << GetNode ()->GetId ());
    }
  else
    {
      m_mobilityModel = model;
    }

  // current mobility
  m_currentSourPosVector.gnAddr = GetNode ()->GetId ();

  //m_currentSourPosVector.Ts = (uint32_t) timeToTaiMilliSecondsSince2004Mod32(Simulator::Now());
  m_currentSourPosVector.Ts = (uint32_t) Simulator::Now ().GetMilliSeconds ();
  m_currentSourPosVector.Lat = (uint32_t) m_mobilityModel->GetPosition ().x;
  m_currentSourPosVector.Lon = (uint32_t) m_mobilityModel->GetPosition ().y;

  m_currentSourPosVector._speed = sqrt (pow (m_mobilityModel->GetVelocity().x, 2) + pow (m_mobilityModel->GetVelocity().y, 2));
  m_currentSourPosVector._heading = 0;

  m_currentSourPosVector.Speed = (uint32_t) round(m_currentSourPosVector._speed / 0.01); // speed as store unit
  m_currentSourPosVector.Heading = (uint16_t) round(m_currentSourPosVector._heading / 0.01);

}

void
V2xClient::SendPacket (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_sendEvent.IsExpired ());

  if ((m_send_socket->Send (packet)) >= 0)
  //if ((m_send_socket->SendTo (packet, 0, m_peerAddress)) >= 0)
    {
      ++m_sentCounter;
    }
  else
    {
      NS_LOG_ERROR ("Error while sending");
    }
}


/*
static int
write_out(const void *buffer, size_t size, void *app_key) {
    FILE *out_fp = app_key;
    size_t wrote = fwrite(buffer, 1, size, out_fp);
    return (wrote == size) ? 0 : -1;
}
*/

void
V2xClient::SendCAM (bool withLowFreq)
{
  NS_LOG_FUNCTION (this);

  AquireMobilityInfo();

/* TODO: fix CAM payload generation (wscript linker error)

  CAM_t *message = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));

  ItsPduHeader_t& header = (message)->header;
  header.protocolVersion = 0;//ItsPduHeader__protocolVersion_currentVersion;
  header.messageID = 0;//ItsPduHeader__messageID_cam;
  header.stationID = 0;

  CoopAwareness_t& cam = (message)->cam;
  cam.generationDeltaTime = 0;
  BasicContainer_t& basic = cam.camParameters.basicContainer;
  HighFrequencyContainer_t& hfc = cam.camParameters.highFrequencyContainer;

  basic.stationType = StationType_passengerCar;
  basic.referencePosition.altitude.altitudeValue = AltitudeValue_oneCentimeter;
  basic.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
  basic.referencePosition.longitude = 0 * Longitude_oneMicrodegreeEast;
  basic.referencePosition.latitude = 0 * Latitude_oneMicrodegreeNorth;
  basic.referencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
  basic.referencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;

  hfc.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

  BasicVehicleContainerHighFrequency& bvc = hfc.choice.basicVehicleContainerHighFrequency;
  bvc.speed.speedValue = 0;
  bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;


  if (withLowFreq)
    {
      LowFrequencyContainer_t*& lfc = cam.camParameters.lowFrequencyContainer;
      lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;

      BasicVehicleContainerLowFrequency& bvc = lfc->choice.basicVehicleContainerLowFrequency;
      bvc.vehicleRole = VehicleRole_default;
      //bvc.exteriorLights.buf = static_cast<uint8_t*>(malloc(1));
      //bvc.exteriorLights.size = 1;
      //bvc.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
      // TODO: add pathHistory

    }

  //NS_LOG_UNCOND(sizeof(message));

  asn_enc_rval_t erv;
  erv = uper_encode(&asn_DEF_CAM, message, write_out, 0);

  if(erv.encoded == -1)
    {
      NS_LOG_UNCOND("Cannot encode CAM");
    }

  uint32_t bytes = erv.encoded / 8;
  bytes += !!(erv.encoded % 8);

  NS_LOG_UNCOND(erv.encoded);

  //ASN_STRUCT_FREE(asn_DEF_CAM, message);

*/

  uint32_t size = 341;
  if (withLowFreq)
    {
      size = 346;
    }

  uint32_t bytes = size / 8;
  bytes += !!(size % 8);

  Ptr<Packet> packet = Create<Packet> (bytes + 9*4 + 4); // packet + commonHeader + btpHeader

  Ptr<GnAddress> resAddress = CreateObject<GnAddress> ();
  //resAddress->Set(GnAddress::BROAD, 1); // nodeId == ID_BROADCAST
  resAddress->Set(m_currentSourPosVector.gnAddr, m_currentSourPosVector.Lat, m_currentSourPosVector.Lon); // nodeId != ID_BROADCAST
  // TODO: GnAddress usage

  GnCommonHeader commonHeader;
  commonHeader.SetSourPosVector(m_currentSourPosVector);
  commonHeader.SetVersion(1);
  commonHeader.SetTrafClass(2);
  commonHeader.SetHopLimit(2);
  commonHeader.SetLength(commonHeader.GetSerializedSize() + packet->GetSize()); //Length of the C2C-CC common network header + Payload
  commonHeader.SetNextHeader(1); // COMMON_HEADER
  packet->AddHeader (commonHeader);

  GnBasicTransportHeader btpHeader;
  btpHeader.SetDestinationPort (PORT_CAM);
  btpHeader.SetSourcePort (PORT_CAM);
  packet->AddHeader (btpHeader);

  m_sendEvent = Simulator::Schedule(Seconds(0), &V2xClient::SendPacket, this, packet);

}

void
V2xClient::checkTriggeringConditions (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_checkEvent.IsExpired ());

  // TODO: fix LTE device state
  if (this->GetNode()->GetDevice(0)->GetObject<LteUeNetDevice> ()->GetRrc()->GetState() == LteUeRrc::CONNECTED_NORMALLY)
    {
      if (m_cam_lastSend == 0) {

	  NS_LOG_DEBUG ("Sending first CAM");
	  SendCAM (false);

	  m_cam_lastSend = Simulator::Now();
	  m_cam_lastSendLowFreq = m_cam_lastSend;
	  m_cam_lastSendSourPosVector = m_currentSourPosVector;

      } else {

	  int64_t deltaTime = (Simulator::Now() - m_cam_lastSend).GetMilliSeconds();
	  int64_t lowFreqDeltaTime = (Simulator::Now() - m_cam_lastSendLowFreq).GetMilliSeconds();

	  if (farEnough(m_cam_lastSendSourPosVector, m_currentSourPosVector)) {
	      m_cam_highDynamics = HIGH_DYNAMICS_CAM_COUNT;
	  }

	  if (deltaTime >= (CAM_INTERVAL_MAX_MS - 2 * CAM_INTERVAL_MIN_MS) ||
	      m_cam_highDynamics > 0) {

	      bool withLowFreq = lowFreqDeltaTime >= CAM_LOW_FREQ_INTERVAL_MS;

	      NS_LOG_DEBUG ("Sending CAM" << (withLowFreq ? " with Low Freq container" : ""));
	      SendCAM (withLowFreq);

	      m_cam_lastSend = Simulator::Now();
	      m_cam_lastSendSourPosVector = m_currentSourPosVector;
	      if (withLowFreq) { m_cam_lastSendLowFreq = m_cam_lastSend; }
	      if (m_cam_highDynamics > 0) { m_cam_highDynamics--; }

	  } else {
	      // NS_LOG_DEBUG("Skip CAM");
	  }
      }
    }
  else
    {
      m_cam_lastSend = Time();
    }

  m_checkEvent = Simulator::Schedule (MilliSeconds (CHECK_CAM_GEN), &V2xClient::checkTriggeringConditions, this);

}

bool
V2xClient::farEnough(GnCommonHeader::LongPositionVector oldPos, GnCommonHeader::LongPositionVector newPos) {
  NS_LOG_FUNCTION (this);

  return fabs(oldPos._heading - newPos._heading) > 4.0 ||
      DistanceInMeters(oldPos.Lat, oldPos.Lon, newPos.Lat, newPos.Lon) > 4.0 ||
      fabs(oldPos._speed - newPos._speed) > 4.0;
}

// Private Members

void
V2xClient::Receive (Ptr<Socket> socket) {
  NS_LOG_FUNCTION (this << socket);

  Ptr<Packet> packet;

  Address from;

  while ((packet = socket->RecvFrom(from))) {

    if (packet->GetSize() == 0) { //EOF
      break;
    }

    /*
     * GeoNetworking Basic Transport Header
     */
    GnBasicTransportHeader btpHeader;
    packet->RemoveHeader (btpHeader);
    //btpHeader.Print(std::cout);

    /*
     * GeoNetworking Common header
     */
    GnCommonHeader commonHeader;
    packet->RemoveHeader (commonHeader);
    //commonHeader.Print(std::cout);

    //NS_LOG_UNCOND("Delay " << Simulator::Now ().GetMilliSeconds() - commonHeader.GetSourPosVector().Ts);

    // reflector
    // TODO: packet forward for flow monitor ?
    if (!m_withCam)
      {

	packet->RemoveAllPacketTags ();
	packet->RemoveAllByteTags ();

	NS_LOG_LOGIC ("Reflectoring packet");

	// attach original headers
	packet->AddHeader (commonHeader);
	packet->AddHeader (btpHeader);

	m_sendEvent = Simulator::Schedule(Seconds(0), &V2xClient::SendPacket, this, packet);

      }
    else
      {
	//Time txTime = milliSecondsMod32ToTime(commonHeader.GetSourPosVector().Ts).GetMilliSeconds();

	NS_LOG_DEBUG ("RX " << packet->GetSize () <<
		     //" bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
		     " bytes from "<< commonHeader.GetSourPosVector().gnAddr <<
		     " Uid: " << packet->GetUid () <<
		     //" TXtime: " << milliSecondsMod32ToTime(commonHeader.GetSourPosVector().Ts).GetMilliSeconds() <<
		     " TXtime: " << commonHeader.GetSourPosVector().Ts <<
		     " RXtime: " << Simulator::Now ().GetMilliSeconds() <<
		     " Delay: " << (Simulator::Now ().GetMilliSeconds() - commonHeader.GetSourPosVector().Ts) << "ms");

	//NS_LOG_UNCOND (DistanceInMeters (0, 0, commonHeader.GetSourPosVector ().Lat, commonHeader.GetSourPosVector ().Lon ) << "m");
	//packet->RemoveAtStart(packet->GetSize());
      }

    /*

    TODO: use LocationTable

    Ptr <LocationTable> ltable = m_node->GetObject<LocationTable> ();
    if (ltable != 0)
      {
	ltable->AddPosEntry (commonHeader);
      }
    */

  }
}

};
