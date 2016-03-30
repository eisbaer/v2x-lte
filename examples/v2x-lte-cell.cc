/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 */
/*
 * Author: Andreas Meitzner
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
//#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
//#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"

// ConfigStore
#include "ns3/config-store.h"
//#include "ns3/gtk-config-store.h"

// FlowMonitor
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

// Gnuplot
#include "ns3/gnuplot.h"


// Netanim
#include "ns3/netanim-module.h"
#include "ns3/animation-interface.h"

// V2x
#include "ns3/v2x-mobility-model.h"
#include "ns3/v2x-client-helper.h"
#include "ns3/v2x-client.h"


using namespace ns3;


NS_LOG_COMPONENT_DEFINE ("V2X LTE");


/*
 * FlowMonitor captures end-to-end Layer3 (IP) traffic, and there is no traffic between a UE and an eNB
 * in the LENA architecture; therefore, we might get 0 throughput from UE to eNB when using FlowMonitor
 */
void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> monitor, Gnuplot2dDataset dataset)
  {
    double tempThroughput = 0.0;
    ns3::BooleanValue flowXml = false;
    monitor->CheckForLostPackets ();
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = monitor->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
      {
	// A tuple: Source-ip, destination-ip, protocol, source-port, destination-port
	Ipv4FlowClassifier::FiveTuple fiveTuple = classifier->FindFlow (stats->first);

	if (fiveTuple.destinationAddress == "7.0.0.2")
	  {
	    std::cout<<"Flow ID: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
	    std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
	    std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
	    std::cout<<"Duration: "<<stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
// 	    std::cout<<"Last Received Packet: "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;

	    tempThroughput = stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024;
	    std::cout<<"Last Received Packet: "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds ---->" << "Throughput: " << tempThroughput << " Kbps" << std::endl;

 	    dataset.Add ((double)stats->second.timeLastRxPacket.GetSeconds (), (double)tempThroughput);
	  }
      }
    Simulator::Schedule (Seconds (1), &ThroughputMonitor, fmhelper, monitor, dataset);

    // Write results to an XML file
    if(flowXml)
      {
	monitor->SerializeToXmlFile ("ThroughputMonitor.xml", true, true);
      }
  }

uint16_t schedulerChoice = 0;

uint32_t numberOfVehicles = 200;

//
uint32_t numberOfVoipUEs = 24;
uint32_t numberOfDownloadUEs = 15;
uint32_t numberOfVideoUEs = 10;
uint32_t numberOfGamingUEs = 6;

uint16_t speedOfVehicles = 50; // default no fading
uint16_t speedOfPedestrians = 3; // default no fading

double moving_bound = 15000;
double distance = 5000;


int
main (int argc, char *argv[])
{
  Time::SetResolution (Time::NS);

  // Command line arguments
  CommandLine cmd;
  cmd.Usage ("V2X LTE.\n"
	      "\n"
  	      "LTE MAC scheduler in V2X scenario.");
  cmd.AddValue ("scheduler", "Scheduler: 0 PF, 1 FD-MT, 2 TD-MT, 3 TTA, 4 FD-BET, 5 TD-BET, 6 RR, 7 FD-TBFQ, 8 TD-TBFQ, 9 PSS", schedulerChoice);
  cmd.AddValue ("numberOfVehicles", "Number of Vehicles", numberOfVehicles);
  cmd.AddValue ("numberOfVoipUEs", "Number of UEs (VoIP)", numberOfVoipUEs);
  cmd.AddValue ("numberOfDownloadUEs", "Number of UEs (Downloading)", numberOfDownloadUEs);
  cmd.AddValue ("numberOfVideoUEs", "Number of UEs (Video)", numberOfVideoUEs);
  cmd.AddValue ("numberOfGamingUEs", "Number of UEs (Gaming)", numberOfGamingUEs);

  // TODO: enable speed values from cmdline
  //cmd.AddValue ("speedOfVehicles", "Speed of Vehicles (kph)", speedOfVehicles);
  //cmd.AddValue ("speedOfUEs", "Speed of Pedestrians (kph)", speedOfPedestrians);

  // ConfigStore setting
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue("scenario.conf"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue("Load"));
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));

  // Setting Path loss Model
  Config::SetDefault ("ns3::LteHelper::PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
  // TODO: Setting Fading Model

  // Settings eNB
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", StringValue ("25"));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", StringValue ("25"));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", StringValue ("100"));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("18100"));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", StringValue ("30"));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", StringValue ("5"));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", StringValue ("320")); // number of UEs

  // Settings UE
  Config::SetDefault ("ns3::LteUePhy::TxPower", StringValue ("23"));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", StringValue ("5"));

  // logging
  //LogLevel level = (LogLevel) (LOG_LEVEL_INFO | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_PREFIX_FUNC);
  // LogComponentEnable ("LteHelper", level);
  // LogComponentEnable ("LteUeMac", level);
  // LogComponentEnable ("LteEnbMac", level);
  // LogComponentEnable("UdpClient", level);
  // LogComponentEnable ("PacketSink", level);

  // ConfigStore
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();
  inputConfig.ConfigureAttributes();

  // Parse command line
  cmd.Parse(argc, argv);

  // Create lteHelper and then epcHelper
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Select LTE scheduler type

  std::string scheduler = "PF";
  //lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler"); // default

  switch (schedulerChoice)
    {
      case (1):
	scheduler = "FD-MT";
	lteHelper->SetSchedulerType ("ns3::FdMtFfMacScheduler");
	break;

      case (2):
	scheduler = "TD-MT";
	lteHelper->SetSchedulerType ("ns3::TdMtFfMacScheduler");
	break;

      case (3):
	scheduler = "TTA";
	lteHelper->SetSchedulerType ("ns3::TtaFfMacScheduler");
	break;

      case (4):
	scheduler = "FD-BET";
	lteHelper->SetSchedulerType ("ns3::FdBetFfMacScheduler");
	break;

      case (5):
	scheduler = "TD-BET";
	lteHelper->SetSchedulerType ("ns3::TdBetFfMacScheduler");
	break;

      case(6):
	scheduler = "RR";
	lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
	break;

      case (7):
	scheduler = "FD-TBFQ";
	lteHelper->SetSchedulerType ("ns3::FdTbfqFfMacScheduler");
	//Parameters for TBFQ scheduler
	lteHelper->SetSchedulerAttribute("DebtLimit", IntegerValue(-625000)); // default value -625000 bytes (-5Mb)
	lteHelper->SetSchedulerAttribute("CreditLimit", UintegerValue(625000)); // default value 625000 bytes (5Mb)
	lteHelper->SetSchedulerAttribute("TokenPoolSize", UintegerValue(1)); // default value 1 byte
	lteHelper->SetSchedulerAttribute("CreditableThreshold", UintegerValue(0)); // default value 0
	break;

      case (8):
	scheduler = "TD-TBFQ";
	lteHelper->SetSchedulerType ("ns3::TdTbfqFfMacScheduler");
	//Parameters for TBFQ scheduler
	lteHelper->SetSchedulerAttribute("DebtLimit", IntegerValue(-625000)); // default value -625000 bytes (-5Mb)
	lteHelper->SetSchedulerAttribute("CreditLimit", UintegerValue(625000)); // default value 625000 bytes (5Mb)
	lteHelper->SetSchedulerAttribute("TokenPoolSize", UintegerValue(1)); // default value 1 byte
	lteHelper->SetSchedulerAttribute("CreditableThreshold", UintegerValue(0)); // default value 0
	break;

      case (9):
	scheduler = "PSS";
	lteHelper->SetSchedulerType ("ns3::PssFfMacScheduler");
	//Parameters for PSS scheduler
	//lteHelper->SetSchedulerAttribute("nMux", UIntegerValue(10)); // the maximum number of UE selected by TD scheduler
	lteHelper->SetSchedulerAttribute("PssFdSchedulerType", StringValue("CoItA")); // PF scheduler type in PSS
	break;

    default:
  	//PF scheduler
  	break;
  };

  // Create a Gateway Node
  Ptr<Node> pgw = epcHelper->GetPgwNode ();
  epcHelper->SetAttribute ("S1uLinkDataRate", DataRateValue (DataRate ("1Gb/s"))); // Data rate
  epcHelper->SetAttribute ("S1uLinkMtu", UintegerValue (1500)); // MTU
  epcHelper->SetAttribute ("S1uLinkDelay", TimeValue (Seconds (0.015))); // Delay

  // Create a eNB
  NodeContainer enbContainer;
  enbContainer.Create(1); // single cell scenario

  // eNB Mobility
  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator (
     "ns3::GridPositionAllocator",
     "MinX", DoubleValue (0.0),  //zero point
     "MinY", DoubleValue (0.0),  //zero point
     "DeltaX", DoubleValue (10000.0),  //distance among eNB nodes
     "DeltaY", DoubleValue (10000.0),
     "GridWidth", UintegerValue (3), //number of nodes on a line
     "LayoutType", StringValue ("RowFirst"));
  enbMobility.Install (enbContainer); // ENB #1 placed at (0.0)
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbContainer);
  NetDeviceContainer::Iterator enbLteDevIt = enbLteDevs.Begin ();
  Vector enbPosition = (*enbLteDevIt)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  enbMobility.Install (remoteHostContainer);

  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));	// Data rate
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));			// MTU
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));		// Delay

  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);		// Attach the remote host to a PGW
  // Set IPv4 address of the gateway Node
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  NS_LOG_UNCOND ("RemoteHost's Ipv4 Address " << remoteHostAddr );

  // Routing of the Internet Host (towards the LTE network or the address of the remote node)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NS_LOG_LOGIC ("Setting up vehicles and UEs");

  // Node Container
  NodeContainer v2xNodes;
  v2xNodes.Create(numberOfVehicles);

  NodeContainer ueNodes;
  uint16_t numOfUeNodes;
  numOfUeNodes = numberOfVoipUEs + numberOfVideoUEs + numberOfGamingUEs + numberOfDownloadUEs;
  ueNodes.Create(numOfUeNodes);

  // Mobility Helper
  MobilityHelper v2xMobility;
  MobilityHelper ueMobility;

  // Install mobility model in UEs

  //nodes are put randomly inside a circle with the central point is (x,y).
  ueMobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
				    "X", DoubleValue (enbPosition.x),
				    "Y", DoubleValue (enbPosition.y),
				    "rho", DoubleValue (distance));  //radius of the circle.

  ueMobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
				    "Mode", StringValue ("Time"),  //change distance and speed based on TIME.
				    "Time", StringValue ("2s"),  //change direction and speed after each 2s.
				    "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.2|Max=0.84]"),  //m/s
				    "Bounds", RectangleValue (Rectangle (-moving_bound, moving_bound, -moving_bound, moving_bound)) );
  ueMobility.Install(ueNodes);

  // Install random speed mobility model in vehicles
  v2xMobility.SetMobilityModel ("ns3::V2xMobilityModel",
				   "Mode", StringValue ("Time"),
				   "Time", StringValue ("40s"),
				   "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30.0]"),
				   "Bounds", RectangleValue (Rectangle (0, 10000, -1000, 1000)));
  v2xMobility.Install(v2xNodes);

  // Create a 3 line grid of vehicles (highway scenario)
  double longitudinalVehicleDistance = 7.0; // m
  double lateralLaneDistance = 3.0; // m

  for (uint16_t i = 0; i < v2xNodes.GetN(); i++)
  {
      if(i % 3 == 0){
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, 0, 0));
      }
      else if(i % 3 == 1){
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, lateralLaneDistance, 0));
      }
      else{
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, lateralLaneDistance*2, 0));
      }
  }

  // Install LTE devices in vehicles and UE(s)
  NetDeviceContainer v2xLteDevs = lteHelper->InstallUeDevice (v2xNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install IP Stack on the vehicles
  internet.Install (v2xNodes);
  Ipv4InterfaceContainer v2xIpInterface;
  v2xIpInterface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (v2xLteDevs));
  for (uint32_t v = 0; v < v2xNodes.GetN (); ++v)
    {
      Ptr<Node> v2xNode = v2xNodes.Get (v);
      Ptr<Ipv4StaticRouting> v2xStaticRouting = ipv4RoutingHelper.GetStaticRouting (v2xNode->GetObject<Ipv4> ());
      v2xStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      NS_LOG_DEBUG ("Vehicle" << v << "'s Ipv4 Address " << v2xIpInterface.GetAddress (v) );
      //Ipv4Address ueIpAddr =  ueNodes.Get (u)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    }

  // Install IP Stack on the UE(s)
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpInterface;
  ueIpInterface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      NS_LOG_DEBUG ("UE" << u << "'s Ipv4 Address " << ueIpInterface.GetAddress (u) );
    }

  // Attach UE(s) to eNB
  for (uint32_t t = 0; t < v2xNodes.GetN (); ++t)
    {
      lteHelper->Attach (v2xLteDevs.Get(t), enbLteDevs.Get(0)); // Attach function takes Interfaces as parameters.
      // side effect: the default EPS bearer will be activated
    }
  for (uint32_t t = 0; t < ueNodes.GetN (); ++t)
    {
      lteHelper->Attach (ueLteDevs.Get(t), enbLteDevs.Get(0));
    }

  NS_LOG_LOGIC ("Setting up applications");

  // randomize start times to avoid simulation artifacts
  Ptr<UniformRandomVariable> startSimTimeSeconds = CreateObject<UniformRandomVariable> ();
  startSimTimeSeconds->SetAttribute ("Min", DoubleValue (0.010));	// 0.010 second
  startSimTimeSeconds->SetAttribute ("Max", DoubleValue (0.050));	// 0.050 second

  // Add V2X Client to vehicles
  V2xClientHelper v2xClient (remoteHostAddr, true);
  ApplicationContainer v2xClientApps = v2xClient.Install (v2xNodes);
  v2xClientApps.Start (Seconds (startSimTimeSeconds->GetValue ()));

  // TODO: choose multicast destination node based on LTE CQI value or place node on the edge of LTE range
  /*
   * Every V2X packet must go through LTE infrastructure (eNB/EPC)
   * Normally the packets are send to Multicast address
   * LENA simulator does not support multicast, so multicast is simulated by a single vehicleUE
   */
  Ipv4Address fakeMulticastDest ("7.0.0.2");

  V2xClientHelper v2xReflector(fakeMulticastDest, false);
  ApplicationContainer v2xServerApps = v2xReflector.Install (remoteHost);
  v2xServerApps.Start (Seconds (0.010));

  // Background traffic
  uint16_t dlPort = 10000;

  for (uint32_t u = 0; u < numOfUeNodes; ++u)
    {
      ++dlPort;

      Ptr<Node> ueNode = ueNodes.Get (u);

      ApplicationContainer clientApps;
      ApplicationContainer serverApps;

      double packetInterval;
      uint32_t packetSize;
      uint16_t localPort;
      EpsBearer bearer;

      // TODO: fix uniform application/traffic assignment
      if (numberOfVoipUEs > 0 && u < numberOfVoipUEs)
	{
	  NS_LOG_INFO ("UE" << u << " VoIP");
	  packetInterval = 20;
	  packetSize = 20;
	  localPort = 5222; // Google Voice
	  bearer = EpsBearer::GBR_CONV_VOICE;
	}
      else if(numberOfVideoUEs > 0 && u >= numberOfVoipUEs && u < (numberOfVoipUEs+numberOfVideoUEs))
	{
	  NS_LOG_INFO ("UE" << u << " Video");
	  packetInterval = 20;
	  packetSize = 1400;
	  localPort = 3478; // Apple Facetime
	  bearer = EpsBearer::GBR_CONV_VIDEO;
	}
      else if(numberOfGamingUEs > 0 && u >= (numberOfVoipUEs+numberOfVideoUEs) && u < (numberOfVoipUEs+numberOfVideoUEs+numberOfGamingUEs))
	{
	  NS_LOG_INFO ("UE" << u << " Gaming");
	  packetInterval = 1;
	  packetSize = 1200;
	  localPort = 3679; // Sony PS
	  bearer = EpsBearer::GBR_CONV_VOICE;
	}
      else
	{
	  NS_LOG_INFO ("UE" << u << " Download");
	  packetInterval = 1;
	  packetSize = 50;
	  localPort = 69; // UDP
	  bearer = EpsBearer::NGBR_VIDEO_TCP_OPERATOR;
	}

      // Config UDP Downlink
      NS_LOG_LOGIC ("Installing UDP Downlink Application(s) for UE " << u);

      UdpClientHelper dlClient (ueIpInterface.GetAddress (u), dlPort);
      dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (packetInterval))); // default Seconds(1.0)
      dlClient.SetAttribute ("PacketSize", UintegerValue (packetSize));
      dlClient.SetAttribute ("RemotePort", UintegerValue (localPort));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (4294967295)); // default (100)
      clientApps.Add (dlClient.Install (remoteHost));

      PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), localPort));
      serverApps.Add (dlPacketSinkHelper.Install (ueNode));

      // Filter downlink local port
      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = dlPort;
      dlpf.localPortEnd = dlPort;
      tft->Add (dlpf);

      // Activate bearer
      lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

      Time startSimTime = Seconds (startSimTimeSeconds->GetValue ());
      serverApps.Start (startSimTime);
      clientApps.Start (startSimTime);
    }

  // Enable traces
  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();

  // Setup for calculating throughput
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));

  // Throughput Monitor
  FlowMonitorHelper fmhelper;
  Ptr<FlowMonitor> fmonitor;
  fmonitor = fmhelper.Install (v2xNodes);
  //fmonitor = fmhelper.Install (ueNodes);
  fmonitor = fmhelper.Install (remoteHostContainer);

  // TODO: enable/disable gnuplot from cmdline

  // Gnuplot parameters
  std::string fileNameWithNoExtension = "TimeVsThroughput";
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  std::string plotTitle               = "Time vs Throughput";
  std::string dataTitle               = "Throughput";

  // Gnuplot - Init the plot
  Gnuplot gnuplot (graphicsFileName);
  gnuplot.SetTitle (plotTitle);
  gnuplot.SetTerminal ("png");					// Make the .PNG file, which the plot file will be when it is used with Gnuplot
  gnuplot.SetLegend ("Time (seconds)", "Throughput (kbps)");   	// Set the labels for each axis
  Gnuplot2dDataset dataset;					// Instantiate the dataset, set its title, and make the points be plotted along with connecting lines.
  dataset.SetTitle (dataTitle);
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);

  NS_LOG_UNCOND("Start Simulation.");
  Simulator::Stop(Seconds(5.0));

  // TODO: enable/disable ThroughputMonitor from cmdline
  ThroughputMonitor (&fmhelper, fmonitor, dataset);

  Simulator::Run();
  NS_LOG_UNCOND("Done.");

  // Gnuplot - write plot
  gnuplot.AddDataset (dataset);
  std::ofstream plotFile (plotFileName.c_str()); // Open the plot file.
  gnuplot.GenerateOutput (plotFile); // Write the plot file.
  plotFile.close (); // Close the plot file.

  Simulator::Destroy();
  return 0;

}
