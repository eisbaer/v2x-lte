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

#include "gen/cam.h"
#include <algorithm>


NS_LOG_COMPONENT_DEFINE ("V2xClient");

namespace ns3 {

  // Source: https://github.com/alexvoronov/geonetworking

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
    .AddTraceSource ("Tx", "A packet is created and send",
		   MakeTraceSourceAccessor (&V2xClient::m_txTrace),
		  "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx", "A packet is received",
		   MakeTraceSourceAccessor (&V2xClient::m_rxTrace),
		  "ns3::Packet::TracedCallback")
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
  m_currentSourPosVector.Alt = (uint32_t) m_mobilityModel->GetPosition ().z;

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
    {
      // Trace
      m_txTrace(packet);
      ++m_sentCounter;
    }
  else
    {
      NS_LOG_ERROR ("Error while sending");
    }
}

typedef std::vector<uint8_t> ByteBuffer;

static int write_buffer(const void* in, std::size_t size, void* out_void)
{
  NS_ASSERT(out_void != nullptr);
  ByteBuffer* out = static_cast<ByteBuffer*>(out_void);
  std::copy_n(static_cast<const uint8_t*>(in), size, std::back_inserter(*out));
  return 0;
}

void
V2xClient::SendCAM (bool withLowFreq)
{
  NS_LOG_FUNCTION (this);

  AquireMobilityInfo();

  CAM_t *message = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
  if(!message) {
      NS_FATAL_ERROR("calloc() failed");
  }
  assert(message); // Infinite memory!

  // Source: https://github.com/bastibl/its-g5-cam

  // initialize struct
  message->header.protocolVersion = protocolVersion_currentVersion;
  message->header.messageID = messageID_cam;
  message->header.stationID = m_currentSourPosVector.gnAddr;

  /// CAM
  message->cam.generationDeltaTime = m_currentSourPosVector.Ts;

  // basicContainer
  message->cam.camParameters.basicContainer.stationType = StationType_passengerCar;

  message->cam.camParameters.basicContainer.referencePosition.latitude = m_currentSourPosVector.Lat;
  message->cam.camParameters.basicContainer.referencePosition.longitude = m_currentSourPosVector.Lon;
  message->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue = m_currentSourPosVector.Alt;
  message->cam.camParameters.basicContainer.referencePosition.altitude.altitudeConfidence = AltitudeConfidence_alt_020_00;

  // highFrequencyContainer
  message->cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue = HeadingValue_wgs84North;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingConfidence = HeadingConfidence_equalOrWithinZeroPointOneDegree;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue = m_currentSourPosVector._speed * SpeedValue_oneCentimeterPerSec;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec * 3;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection = DriveDirection_forward;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue = VehicleLengthValue_tenCentimeters * 48; // 4,77 m
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_noTrailerPresent;
  message->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth = VehicleWidth_tenCentimeters * 19; // 1,83 m

  // LowFrequencyContainer
  if (withLowFreq)
    {
      LowFrequencyContainer_t *lfc = static_cast<LowFrequencyContainer_t*>(calloc(1, sizeof(LowFrequencyContainer_t)));
      if(!lfc) {
	  NS_FATAL_ERROR("calloc() failed");
      }
      assert(lfc);

      lfc->present = LowFrequencyContainer_PR_basicVehicleContainerLowFrequency;
      lfc->choice.basicVehicleContainerLowFrequency.vehicleRole = VehicleRole_default;
      lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.buf = static_cast<uint8_t*>(calloc(1, sizeof(uint8_t)));
      lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.size = 1;
      lfc->choice.basicVehicleContainerLowFrequency.exteriorLights.buf[0] |= 1 << (7 - ExteriorLights_daytimeRunningLightsOn);
      // TODO: pathHistory
      //lfc->choice.basicVehicleContainerLowFrequency.pathHistory

      message->cam.camParameters.lowFrequencyContainer = lfc;
    }


  ByteBuffer buffer;

  //uint8_t *buffer = new uint8_t[680];

  asn_enc_rval_t ec;
  ec = uper_encode(&asn_DEF_CAM, message, write_buffer, &buffer);

  if(ec.encoded < 0)
    {
      const char* failed_type = ec.failed_type ? ec.failed_type->name : "unknown";
      NS_FATAL_ERROR("Unaligned PER encoding of type " << asn_DEF_CAM.name << " failed because of " << failed_type << " sub-type");
    }

  uint32_t bytes = ec.encoded / 8;
  bytes += !!(ec.encoded % 8);

  //NS_LOG_DEBUG("CAM " << ec.encoded << " bit " << bytes);


  //asn_fprint(stdout, &asn_DEF_CAM, message);


  ASN_STRUCT_FREE(asn_DEF_CAM, message);


  Ptr<Packet> packet = Create<Packet> (buffer.data(), bytes);

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

    // Trace
    m_rxTrace(packet);

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

	uint32_t packetSize = packet->GetSize ();

	CAM_t *message = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
	if(!message) {
	    NS_FATAL_ERROR("calloc() failed");
	}

	uint8_t *buffer=new uint8_t[packetSize + 1];
	packet->CopyData (buffer, packetSize);

	asn_codec_ctx_t ctx;
	asn_dec_rval_t dc = uper_decode_complete(&ctx, &asn_DEF_CAM, (void**)&message, buffer, packetSize);

	if (dc.code != RC_OK)
	  {
	    NS_FATAL_ERROR("Unaligned PER decoding failed");
	  }

	//asn_fprint(stdout, &asn_DEF_CAM, message);

	if (message->cam.camParameters.lowFrequencyContainer != nullptr)
	  {
	    NS_LOG_DEBUG("LowFrequencyContainer");
	  }

	if (message->cam.camParameters.specialVehicleContainer != nullptr)
	  {
	    NS_LOG_DEBUG("SpecialVehicleContainer");
	  }

	NS_LOG_DEBUG ("RX " << packetSize <<
		     " bytes from "<< commonHeader.GetSourPosVector().gnAddr <<
		     " Uid: " << packet->GetUid () <<
		     //" TXtime: " << milliSecondsMod32ToTime(commonHeader.GetSourPosVector().Ts).GetMilliSeconds() <<
		     " TXtime: " << commonHeader.GetSourPosVector().Ts <<
		     " RXtime: " << Simulator::Now ().GetMilliSeconds() <<
		     " Delay: " << (Simulator::Now ().GetMilliSeconds() - commonHeader.GetSourPosVector().Ts) << "ms");
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
