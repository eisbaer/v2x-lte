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
#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/mac-stats-calculator.h"
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

// Histogram
#include "ns3/histogram.h"

// Gnuplot
#include "ns3/gnuplot.h"


// Netanim
#include "ns3/netanim-module.h"

// V2x
#include "ns3/v2x-mobility-model.h"
#include "ns3/v2x-client-helper.h"
#include "ns3/v2x-client.h"

#include "ns3/gn-basic-transport-header.h"
#include "ns3/gn-common-header.h"

#include <map>
#include <string>
#include <iomanip>
#include <ctime>


using namespace ns3;


NS_LOG_COMPONENT_DEFINE ("V2X LTE");

FlowMonitor::FlowStats v2xStats;

typedef std::map<uint16_t,Gnuplot2dDataset> Datasets;
Datasets m_Datasets;


/*
 * FlowMonitor captures end-to-end Layer3 (IP) traffic, and there is no traffic between a UE and an eNB
 * in the LENA architecture; therefore, we might get 0 throughput from UE to eNB when using FlowMonitor
 */
void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> monitor, Datasets datasets)
  {
    double tempThroughput = 0.0;
    monitor->CheckForLostPackets ();
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = monitor->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());

    double voipThroughputSum = 0.0;
    double videoThroughputSum = 0.0;
    double gamingThroughputSum = 0.0;
    double downloadThroughputSum = 0.0;

    uint64_t voipThroughputCount = 0;
    uint64_t videoThroughputCount = 0;
    uint64_t gamingThroughputCount = 0;
    uint64_t downloadThroughputCount = 0;

    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
      {
	// A tuple: Source-ip, destination-ip, protocol, source-port, destination-port
	Ipv4FlowClassifier::FiveTuple fiveTuple = classifier->FindFlow (stats->first);

	if (fiveTuple.destinationPort != 2001) // hide V2X remoteHost flow
	  {
	    NS_LOG_UNCOND ("Port: " << fiveTuple.destinationPort);
	    NS_LOG_UNCOND ("Flow ID: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress);
	    NS_LOG_UNCOND ("Tx Packets = " << stats->second.txPackets);
	    NS_LOG_UNCOND ("Rx Packets = " << stats->second.rxPackets);
	    NS_LOG_UNCOND ("Duration: "<< stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds());

	    tempThroughput = stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds() - stats->second.timeFirstTxPacket.GetSeconds())/1024;
	    NS_LOG_UNCOND ("Last Received Packet: "<< stats->second.timeLastRxPacket.GetSeconds() <<" Seconds");
	    NS_LOG_UNCOND ("Throughput: " << tempThroughput << " Kbps");

	    Time delay;
	    double pdr = 0.0;
	    if (stats->second.rxPackets > 0)
	      {
		delay = stats->second.delaySum / stats->second.rxPackets;
		pdr = (double)stats->second.rxPackets / (double)stats->second.txPackets;
	      }

	    NS_LOG_UNCOND ("Delay: " << delay.GetMilliSeconds()  << " ms");
	    NS_LOG_UNCOND ("PDR: " << pdr );
	    NS_LOG_UNCOND ("");

	    if (fiveTuple.destinationPort == 5222)
	      {
		voipThroughputSum += tempThroughput;
		++voipThroughputCount;
	      }
	    else if(fiveTuple.destinationPort == 3478)
	      {
		videoThroughputSum += tempThroughput;
		++videoThroughputCount;
	      }
	    else if(fiveTuple.destinationPort == 3679)
	      {
		gamingThroughputSum += tempThroughput;
		++gamingThroughputCount;
	      }
	    else if(fiveTuple.destinationPort == 69)
	      {
		downloadThroughputSum += tempThroughput;
		++downloadThroughputCount;
	      }

	    /*
	    // dataset
	    for (std::map<uint16_t, Gnuplot2dDataset>::iterator dataset = datasets.begin(); dataset != datasets.end(); ++dataset)
	      {
		if (dataset->first == fiveTuple.destinationPort)
		  {
		     dataset->second.Add ((double)Simulator::Now ().GetSeconds (), (double)tempThroughput);
		  }
	      }
	    */
	    //dataset.Add ((double)stats->second.timeLastRxPacket.GetSeconds (), (double)tempThroughput);
	  }
      }



    // V2X Flow
    if (v2xStats.txPackets > 0)
      {
	// V2x
	NS_LOG_UNCOND ("Flow ID: V2X #########################");
	NS_LOG_UNCOND ("Tx Packets = " << v2xStats.txPackets);
	NS_LOG_UNCOND ("Rx Packets = " << v2xStats.rxPackets);
	NS_LOG_UNCOND ("Duration: "<< v2xStats.timeLastRxPacket.GetSeconds() - v2xStats.timeFirstTxPacket.GetSeconds());

	double throughput = v2xStats.rxBytes * 8.0 / (v2xStats.timeLastRxPacket.GetSeconds() - v2xStats.timeFirstTxPacket.GetSeconds())/1024;
	NS_LOG_UNCOND ("Last Received Packet: "<< v2xStats.timeLastRxPacket.GetSeconds() <<" Seconds");
	NS_LOG_UNCOND ("Throughput: " << throughput << " Kbps");

	Time delay;
	double pdr = 0.0;
	if (v2xStats.rxPackets > 0)
	  {
	    delay = v2xStats.delaySum / v2xStats.rxPackets;
	    pdr = (double)v2xStats.rxPackets / (double)v2xStats.txPackets;
	  }

	NS_LOG_UNCOND ("Delay: " << delay.GetMilliSeconds()  << " ms");
	NS_LOG_UNCOND ("PDR: " << pdr );
	NS_LOG_UNCOND ("");
      }

    // gnuplot
    for (std::map<uint16_t, Gnuplot2dDataset>::iterator dataset = datasets.begin(); dataset != datasets.end(); ++dataset)
      {
	if (dataset->first == 5222)
	  {
	     dataset->second.Add ((double)Simulator::Now ().GetSeconds (), (double)(voipThroughputSum/voipThroughputCount));
	  }
	else if (dataset->first == 3478)
	  {
	     dataset->second.Add ((double)Simulator::Now ().GetSeconds (), (double)(videoThroughputSum/videoThroughputCount));
	  }
	else if (dataset->first == 3679)
	  {
	     dataset->second.Add ((double)Simulator::Now ().GetSeconds (), (double)(gamingThroughputSum/gamingThroughputCount));
	  }
	else if (dataset->first == 69)
	  {
	     dataset->second.Add ((double)Simulator::Now ().GetSeconds (), (double)(downloadThroughputSum/downloadThroughputCount));
	  }
      }

    Simulator::Schedule (Seconds (1.0), &ThroughputMonitor, fmhelper, monitor, datasets);

  }

void
serializeHistogram(std::string name, Histogram hist, const std::string& title){
    std::stringstream fname;
    std::stringstream fpname;
    fpname << name << ".plt";
    fname << name << ".png";

    Gnuplot plot (fname.str ());
    plot.SetTitle (title);
    plot.SetTerminal ("png");
    plot.SetLegend (title + " (ms)", "");
    Gnuplot2dDataset ds;
    ds.SetTitle (title);
    ds.SetStyle (Gnuplot2dDataset::HISTEPS);
    uint32_t bins = hist.GetNBins ();
    for (uint32_t i=0; i < bins; i++){
        ds.Add (hist.GetBinStart (i), hist.GetBinCount (i));
    }
    plot.AddDataset (ds);
    std::ofstream plotFile (fpname.str ().c_str ());
    plot.GenerateOutput (plotFile);
    plotFile.close();
}

static void
SinkTx (Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy();

  GnBasicTransportHeader btpHeader;
  packet->RemoveHeader(btpHeader);

  GnCommonHeader commonHeader;
  packet->RemoveHeader (commonHeader);

  uint32_t packetSize = p->GetSize();
  Time now = Simulator::Now ();

  v2xStats.txBytes += packetSize;
  v2xStats.txPackets++;
  if (v2xStats.txPackets == 1)
    {
      v2xStats.timeFirstTxPacket = now;
    }
  v2xStats.timeLastTxPacket = now;

  NS_LOG_INFO ("TX " << p->GetSize () <<
	       //" bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
	       " bytes from "<< commonHeader.GetSourPosVector().gnAddr <<
	       " Uid: " << p->GetUid () <<
	       " TXtime: " << commonHeader.GetSourPosVector().Ts );
}

static void
SinkRx (Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy();

  GnBasicTransportHeader btpHeader;
  packet->RemoveHeader(btpHeader);

  GnCommonHeader commonHeader;
  packet->RemoveHeader (commonHeader);

  uint32_t packetSize = p->GetSize();
  Time now = Simulator::Now ();
  Time delay = MilliSeconds(now.GetMilliSeconds() - commonHeader.GetSourPosVector().Ts);

  v2xStats.delaySum += delay;
  v2xStats.delayHistogram.AddValue (delay.GetMilliSeconds ());
  if (v2xStats.rxPackets > 0 )
    {
      Time jitter = v2xStats.lastDelay - delay;
      if (jitter > Seconds (0))
        {
	  v2xStats.jitterSum += jitter;
	  v2xStats.jitterHistogram.AddValue (jitter.GetMilliSeconds ());
        }
      else
        {
          v2xStats.jitterSum -= jitter;
          v2xStats.jitterHistogram.AddValue (-jitter.GetMilliSeconds ());
        }
    }
  v2xStats.lastDelay = delay;

  v2xStats.rxBytes += packetSize;
  v2xStats.packetSizeHistogram.AddValue ((double) packetSize);
  v2xStats.rxPackets++;
  if (v2xStats.rxPackets == 1)
    {
      v2xStats.timeFirstRxPacket = now;
    }
//  else
//    {
//      // measure possible flow interruptions
//      Time interArrivalTime = now - v2xStats.timeLastRxPacket;
//      if (interArrivalTime > m_flowInterruptionsMinTime)
//        {
//          v2xStats.flowInterruptionsHistogram.AddValue (interArrivalTime.GetSeconds ());
//        }
//    }
  v2xStats.timeLastRxPacket = now;

  NS_LOG_INFO ("RX " << packet->GetSize () <<
	       //" bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
	       " bytes from "<< commonHeader.GetSourPosVector().gnAddr <<
	       " Uid: " << packet->GetUid () <<
	       " TXtime: " << commonHeader.GetSourPosVector().Ts <<
	       " RXtime: " << Simulator::Now ().GetMilliSeconds() <<
	       " Delay: " << (Simulator::Now ().GetMilliSeconds() - commonHeader.GetSourPosVector().Ts) << "ms");

}

double duration = 5.0;

uint16_t schedulerChoice = 0;

uint32_t numberOfVehicles = 20;

uint32_t numberOfVoipUEs = 24;
uint32_t numberOfDownloadUEs = 15;
uint32_t numberOfVideoUEs = 10;
uint32_t numberOfGamingUEs = 6;

uint16_t speedOfVehicles = 100;
uint16_t speedOfPedestrians = 3;

double distance = 1500;
double moving_bound = 10000;


std::string
write_speed(int kph) {

  double speed = double (kph)*1000/3600; //kmph to meter per second.
  std::stringstream mss;
  mss << speed;
  std::string ms = mss.str();
  return ms;
}


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
  cmd.AddValue ("distance", "Max. UE distance (m)", distance);
  cmd.AddValue ("duration", "Simulation time (s)", duration);
  cmd.AddValue ("speedOfVehicles", "Speed of Vehicles (kph)", speedOfVehicles);
  cmd.AddValue ("speedOfUEs", "Speed of Pedestrians (kph)", speedOfPedestrians);

  uint16_t numOfUeNodes;
  numOfUeNodes = numberOfVoipUEs + numberOfVideoUEs + numberOfGamingUEs + numberOfDownloadUEs;


  // ConfigStore setting
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue("scenario.conf"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue("Load"));
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));
  Config::SetDefault ("ns3::LteAmc::Ber", DoubleValue (0.00005));

  // Settings eNB
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", StringValue ("25"));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", StringValue ("25"));
  Config::SetDefault ("ns3::LteEnbNetDevice::DlEarfcn", StringValue ("100"));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("18100"));
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", StringValue ("30"));
  Config::SetDefault ("ns3::LteEnbPhy::NoiseFigure", StringValue ("5"));
  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", StringValue ("320")); // number of UEs

  // disable phy error
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));

  // Settings UE
  Config::SetDefault ("ns3::LteUePhy::TxPower", StringValue ("23"));
  Config::SetDefault ("ns3::LteUePhy::NoiseFigure", StringValue ("5"));


  // logging
  //LogLevel level = (LogLevel) (LOG_LEVEL_INFO | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_PREFIX_FUNC);
  // LogComponentEnable ("LteHelper", level);
  // LogComponentEnable ("LteUeMac", level);
  // LogComponentEnable ("LteEnbMac", level);
  // LogComponentEnable ("UdpClient", level);
  // LogComponentEnable ("PacketSink", level);

  // ConfigStore
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();
  inputConfig.ConfigureAttributes();

  // Parse command line
  cmd.Parse(argc, argv);

  // this tag is then appended to all filenames
  std::ostringstream tag;
  tag << "_distance" << std::setw (3) << std::setfill ('0') << distance
      << "_numV2Xs" << std::setw (3) << std::setfill ('0')  << numberOfVehicles
      << "_speedV2Xs" << std::setw (3) << std::setfill ('0')  << speedOfVehicles
      << "_numUEs" << std::setw (3) << std::setfill ('0')  << numOfUeNodes
      << "_speedUEs" << std::setw (3) << std::setfill ('0')  << speedOfPedestrians;

  std::string scheduler = "PF";

  switch (schedulerChoice)
    {
      case (1):
	scheduler = "FD-MT";
	break;

      case (2):
	scheduler = "TD-MT";
	break;

      case (3):
	scheduler = "TTA";
	break;

      case (4):
	scheduler = "FD-BET";
	break;

      case (5):
	scheduler = "TD-BET";
	break;

      case(6):
	scheduler = "RR";
	break;

      case (7):
	scheduler = "FD-TBFQ";
	break;

      case (8):
	scheduler = "TD-TBFQ";
	break;

      case (9):
	scheduler = "PSS";
	break;

    default:
  	//PF scheduler
  	break;
  };

  tag << "_"  << scheduler
      << "_sec"  << std::setw (3) << std::setfill ('0')  << duration;

  NS_LOG_UNCOND ("Simulation: " << tag.str());


  std::string outputpath = "results" ;
  Config::SetDefault ("ns3::MacStatsCalculator::DlOutputFilename", StringValue (outputpath+"/DlMacStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::MacStatsCalculator::UlOutputFilename", StringValue (outputpath+"/UlMacStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::DlRlcOutputFilename", StringValue (outputpath+"/DlRlcStats" + tag.str() + ".txt"));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::UlRlcOutputFilename", StringValue (outputpath+"/UlRlcStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::DlPdcpOutputFilename", StringValue (outputpath+"/DlPdcpStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::RadioBearerStatsCalculator::UlPdcpOutputFilename", StringValue (outputpath+"/UlPdcpStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::PhyStatsCalculator::DlRsrpSinrFilename", StringValue (outputpath+"/DlRsrpSinrStats"+ tag.str() +".txt"));
  Config::SetDefault ("ns3::PhyStatsCalculator::UlSinrFilename", StringValue (outputpath+"/UlSinrStats"+ tag.str() +".txt"));




  // Create lteHelper and then epcHelper
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Pathloss Model
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));

/*
  // Fading model
  lteHelper->SetAttribute ("FadingModel", StringValue ("ns3::TraceFadingLossModel"));


  std::ifstream ifTraceFile;
  ifTraceFile.open ("../../src/v2x-lte/model/fading-traces/fading_trace_EPA_3kmph.fad", std::ifstream::in);
  if (ifTraceFile.good ())
   {
     // script launched by test.py
     lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("../../src/v2x-lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
   }
  else
   {
     // script launched as an example
     lteHelper->SetFadingModelAttribute ("TraceFilename", StringValue ("src/v2x-lte/model/fading-traces/fading_trace_EPA_3kmph.fad"));
   }

  // these parameters have to setted only in case of the trace format
  // differs from the standard one, that is
  // - 10 seconds length trace
  // - 10,000 samples
  // - 0.5 seconds for window size
  // - 100 RB
  lteHelper->SetFadingModelAttribute ("TraceLength", TimeValue (Seconds (10.0)));
  lteHelper->SetFadingModelAttribute ("SamplesNum", UintegerValue (10000));
  lteHelper->SetFadingModelAttribute ("WindowSize", TimeValue (Seconds (0.5)));
  lteHelper->SetFadingModelAttribute ("RbNum", UintegerValue (100));
 */

  // Select LTE scheduler type

  //std::string scheduler = "PF";
  //lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler"); // default

  switch (schedulerChoice)
    {
      case (1):
	lteHelper->SetSchedulerType ("ns3::FdMtFfMacScheduler");
	break;

      case (2):
	lteHelper->SetSchedulerType ("ns3::TdMtFfMacScheduler");
	break;

      case (3):
	lteHelper->SetSchedulerType ("ns3::TtaFfMacScheduler");
	break;

      case (4):
	lteHelper->SetSchedulerType ("ns3::FdBetFfMacScheduler");
	break;

      case (5):
	lteHelper->SetSchedulerType ("ns3::TdBetFfMacScheduler");
	break;

      case(6):
	lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
	break;

      case (7):
	lteHelper->SetSchedulerType ("ns3::FdTbfqFfMacScheduler");
	//Parameters for TBFQ scheduler
	lteHelper->SetSchedulerAttribute("DebtLimit", IntegerValue(-625000)); // default value -625000 bytes (-5Mb)
	lteHelper->SetSchedulerAttribute("CreditLimit", UintegerValue(625000)); // default value 625000 bytes (5Mb)
	lteHelper->SetSchedulerAttribute("TokenPoolSize", UintegerValue(1)); // default value 1 byte
	lteHelper->SetSchedulerAttribute("CreditableThreshold", UintegerValue(0)); // default value 0
	break;

      case (8):
	lteHelper->SetSchedulerType ("ns3::TdTbfqFfMacScheduler");
	//Parameters for TBFQ scheduler
	lteHelper->SetSchedulerAttribute("DebtLimit", IntegerValue(-625000)); // default value -625000 bytes (-5Mb)
	lteHelper->SetSchedulerAttribute("CreditLimit", UintegerValue(625000)); // default value 625000 bytes (5Mb)
	lteHelper->SetSchedulerAttribute("TokenPoolSize", UintegerValue(1)); // default value 1 byte
	lteHelper->SetSchedulerAttribute("CreditableThreshold", UintegerValue(0)); // default value 0
	break;

      case (9):
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
     "MinX", DoubleValue (distance),
     "MinY", DoubleValue (distance),
     "DeltaX", DoubleValue (0.0),  //distance among nodes
     "DeltaY", DoubleValue (0.0),
     "GridWidth", UintegerValue (10), //number of nodes on a line
     "LayoutType", StringValue ("RowFirst"));
  enbMobility.Install (enbContainer); // ENB #1 placed at (0.0)
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbContainer);
  NetDeviceContainer::Iterator enbLteDevIt = enbLteDevs.Begin ();
  Vector enbPosition = (*enbLteDevIt)->GetNode ()->GetObject<MobilityModel> ()->GetPosition ();

  enbMobility.Install (pgw);

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
				    //"Speed", StringValue ("ns3::UniformRandomVariable[Min=0.2|Max=0.84]"),  //m/s
				    "Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max="+write_speed(speedOfPedestrians)+"]"),
				    "Bounds", RectangleValue (Rectangle (-moving_bound, moving_bound, -moving_bound, moving_bound)) );
  ueMobility.Install(ueNodes);

  // Install random speed mobility model in vehicles
  v2xMobility.SetMobilityModel ("ns3::V2xMobilityModel",
				   "Mode", StringValue ("Time"),
				   "Time", StringValue ("4s"),
				   //"Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30.0]"),
				   "Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max="+write_speed(speedOfVehicles)+"]"),
				   "Bounds", RectangleValue (Rectangle (-moving_bound, moving_bound, -moving_bound, moving_bound)));
  v2xMobility.Install(v2xNodes);

  // Create a 3 line grid of vehicles (highway scenario)
  double longitudinalVehicleDistance = (double)(speedOfVehicles) * 1000/3600 * 2; // velocity [m/s] * 2[s]
  double lateralLaneDistance = 3.0; // m

  //double startX = 0;
  double startY = distance/4;

  for (uint16_t i = 0; i < v2xNodes.GetN(); i++)
  {
      if(i % 3 == 0){
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, startY + 0, 1.5));
      }
      else if(i % 3 == 1){
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, startY + lateralLaneDistance, 1.5));
      }
      else{
	  v2xNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (i*longitudinalVehicleDistance, startY + (lateralLaneDistance*2), 1.5));
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

  for (uint32_t v = 0; v < v2xNodes.GetN (); ++v)
    {
      // Filter downlink local port
      Ptr<EpcTft> tft = Create<EpcTft> ();
      EpcTft::PacketFilter dlpf;
      dlpf.localPortStart = 2001;
      dlpf.localPortEnd = 2001;
      tft->Add (dlpf);

      GbrQosInformation qos;

      qos.gbrDl = 32000; // Downlink GBR
      qos.gbrUl = 32000; // Uplink GBR
      qos.mbrDl = qos.gbrDl; // Downlink MBR
      qos.mbrUl = qos.gbrUl; // Uplink MBR
      EpsBearer bearer (EpsBearer::NGBR_VOICE_VIDEO_GAMING, qos);

      // Activate bearer
      lteHelper->ActivateDedicatedEpsBearer (v2xLteDevs.Get (v), bearer, tft);
    }

  // "+std::to_string(idSinkNode)+"
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::V2xClient/Tx", MakeCallback(&SinkTx));
  Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::V2xClient/Rx", MakeCallback(&SinkRx));

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
      EpsBearer::Qci qci;

      // TODO: fix uniform application/traffic assignment
      if (numberOfVoipUEs > 0 && u < numberOfVoipUEs)
	{
	  NS_LOG_INFO ("UE" << u << " VoIP");
	  packetInterval = 20;
	  packetSize = 20;
	  localPort = 5222; // Google Voice
	  qci = EpsBearer::GBR_CONV_VOICE;
	}
      else if(numberOfVideoUEs > 0 && u >= numberOfVoipUEs && u < (numberOfVoipUEs+numberOfVideoUEs))
	{
	  NS_LOG_INFO ("UE" << u << " Video");
	  packetInterval = 20;
	  packetSize = 1400;
	  localPort = 3478; // Apple Facetime
	  qci = EpsBearer::GBR_CONV_VIDEO;
	}
      else if(numberOfGamingUEs > 0 && u >= (numberOfVoipUEs+numberOfVideoUEs) && u < (numberOfVoipUEs+numberOfVideoUEs+numberOfGamingUEs))
	{
	  NS_LOG_INFO ("UE" << u << " Gaming");
	  packetInterval = 1;
	  packetSize = 1200;
	  localPort = 3679; // Sony PS
	  qci = EpsBearer::GBR_CONV_VOICE;
	}
      else
	{
	  NS_LOG_INFO ("UE" << u << " Download");
	  packetInterval = 1;
	  packetSize = 50;
	  localPort = 69; // UDP
	  qci = EpsBearer::NGBR_VIDEO_TCP_OPERATOR;
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

      GbrQosInformation qos;

      //(packetSize [bytes]/RTT[seconds]) * 8 [bits/byte] = Max single tcp throughput in (bps)

      // NS_LOG_DEBUG( "Port" << localPort << ": " << (packetSize/ (packetInterval/1000)) * 8 << " bps" );

      qos.gbrDl = (packetSize/ (packetInterval/1000)) * 8; // Downlink GBR
      qos.gbrUl = (packetSize/ (packetInterval/1000)) * 8; // Uplink GBR
      qos.mbrDl = qos.gbrDl; // Downlink MBR
      qos.mbrUl = qos.gbrUl; // Uplink MBR
      EpsBearer bearer (qci, qos);

      // Activate bearer
      lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

      Time startSimTime = Seconds (startSimTimeSeconds->GetValue ());
      serverApps.Start (startSimTime);
      clientApps.Start (startSimTime);
    }


  AnimationInterface anim (outputpath+"/animation" + tag.str() + ".xml");
  anim.EnablePacketMetadata (); // Optional
  anim.SetMaxPktsPerTraceFile(std::numeric_limits<uint64_t>::max());

  anim.UpdateNodeDescription(remoteHost, "Remote Host");
  anim.UpdateNodeDescription(enbContainer.Get(0), "eNB");

  // Enable traces
  //lteHelper->EnablePhyTraces ();
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
  //fmonitor = fmhelper.Install (v2xNodes);
  fmonitor = fmhelper.Install (ueNodes);
  fmonitor = fmhelper.Install (remoteHostContainer);

  // TODO: enable/disable gnuplot from cmdline

  // Gnuplot parameters
  std::string fileNameWithNoExtension = outputpath + "/Throughput" + tag.str();
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  std::string plotTitle               = "Time vs Throughput";
  std::string dataTitle               = "Throughput";

  // Gnuplot - Init the plot
  Gnuplot gnuplot (graphicsFileName);
  gnuplot.SetTitle (plotTitle);
  gnuplot.SetTerminal ("png");					// Make the .PNG file, which the plot file will be when it is used with Gnuplot
  gnuplot.SetLegend ("Time (seconds)", "Throughput (kbps)");   	// Set the labels for each axis

  Gnuplot2dDataset dataset1;					// Instantiate the dataset, set its title, and make the points be plotted along with connecting lines.
  dataset1.SetTitle("Download");
  dataset1.Add (0.0, 0.0);

  Gnuplot2dDataset dataset2;
  dataset2.SetTitle("VoIP");
  dataset2.Add (0.0, 0.0);

  Gnuplot2dDataset dataset3;
  dataset3.SetTitle("Video");
  dataset3.Add (0.0, 0.0);

  Gnuplot2dDataset dataset4;
  dataset4.SetTitle("Gaming");
  dataset4.Add (0.0, 0.0);

  m_Datasets.insert ( std::pair<int,Gnuplot2dDataset>(69, dataset1) );
  m_Datasets.insert ( std::pair<int,Gnuplot2dDataset>(5222, dataset2) );
  m_Datasets.insert ( std::pair<int,Gnuplot2dDataset>(3478, dataset3) );
  m_Datasets.insert ( std::pair<int,Gnuplot2dDataset>(3679, dataset4) );


  NS_LOG_UNCOND("Start Simulation.");
  Simulator::Stop(Seconds(duration));

  // TODO: enable/disable ThroughputMonitor from cmdline
  ThroughputMonitor (&fmhelper, fmonitor, m_Datasets);

  Simulator::Run();
  NS_LOG_UNCOND("Done.");


  // Gnuplot - write plot
  //gnuplot.AddDataset (dataset);

  for (std::map<uint16_t, Gnuplot2dDataset>::iterator dataset = m_Datasets.begin(); dataset != m_Datasets.end(); ++dataset)
    {
      Gnuplot2dDataset ds = dataset->second;
      ds.SetStyle (Gnuplot2dDataset::LINES_POINTS);

      gnuplot.AddDataset (ds);
    }

  std::ofstream plotFile (plotFileName.c_str()); // Open the plot file.
  gnuplot.GenerateOutput (plotFile); // Write the plot file.
  plotFile.close (); // Close the plot file.


  // hist
  if (v2xStats.rxPackets > 0)
    {
      serializeHistogram(outputpath + "/v2x-delay" + tag.str(), v2xStats.delayHistogram, "Delay");
      serializeHistogram(outputpath + "/v2x-jitter" + tag.str(), v2xStats.jitterHistogram, "Jitter");
      //serializeHistogram("v2x-packetSize", v2xStats.packetSizeHistogram);
    }

  Simulator::Destroy();
  return 0;

}
