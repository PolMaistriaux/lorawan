#include "ns3/callback.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("AlohaThroughput");

// Network settings
int nDevices = 10;
int nGateways = 1;
int packetSize = 50;
double radius = 7500;
double simulationTime = 60;
double appPeriod = 20;
int logProfile = 2;

// Channel model
bool realisticChannelModel = false;

std::vector<int> packetsSent (6, 0);
std::vector<int> packetsReceived (6, 0);
std::vector<int> packetsInterference (6, 0);
std::vector<int> packetsUnderSensitivity (6, 0);
std::vector<int> packetDutyCycle (6, 0);


void
OnTransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  packetsSent.at(tag.GetSpreadingFactor()-7)++;
}

void
OnPacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  packetsReceived.at(tag.GetSpreadingFactor()-7)++;
}

void
OnPacketUnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  packetsUnderSensitivity.at(tag.GetSpreadingFactor()-7)++;
}

void
OnPacketInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  packetsInterference.at(tag.GetSpreadingFactor()-7)++;
}

void
OnPacketDutyCycle (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  packetDutyCycle.at(tag.GetSpreadingFactor()-7)++;
}

int
main (int argc, char *argv[])
{

  std::string interferenceMatrix = "goursaud";

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("simulationTime", "Simulation Time", simulationTime);
  cmd.AddValue ("radius", "Radius of the deployment", radius);
  cmd.AddValue ("appPeriod", "Time in seconds between to packet at a node", appPeriod);
  cmd.AddValue ("logProfile", "Only track sent and received packet", logProfile);
  cmd.AddValue ("packetSize", "Payload size if the packets", packetSize);
  cmd.Parse (argc, argv);

  // Set up logging
  LogComponentEnable ("AlohaThroughput", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ERROR);

  // Make all devices use SF7 (i.e., DR0)
  Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (0));

  if (interferenceMatrix == "aloha")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::ALOHA;
  }
  else if (interferenceMatrix == "goursaud")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::GOURSAUD;
  }

  /***********
   *  Setup  *
   ***********/

  // Mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),
                                 "X", DoubleValue (0.0), "Y", DoubleValue (0.0));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object
  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  if (realisticChannelModel)
    {
      // Create the correlated shadowing component
      Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
          CreateObject<CorrelatedShadowingPropagationLossModel> ();

      // Aggregate shadowing to the logdistance loss
      loss->SetNext (shadowing);
    }

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

  /************************
   *  Create the helpers  *
   ************************/

  // Create the LoraPhyHelper
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();
  macHelper.SetRegion (LorawanMacHelper::ALOHA);

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();
  helper.EnablePacketTracking (); // Output filename

  //Create the NetworkServerHelper
  NetworkServerHelper nsHelper = NetworkServerHelper ();

  //Create the ForwarderHelper
  ForwarderHelper forHelper = ForwarderHelper ();

  /************************
   *  Create End Devices  *
   ************************/

  // Create a set of nodes
  NodeContainer endDevices;
  endDevices.Create (nDevices);

  // Assign a mobility model to each node
  mobility.Install (endDevices);

  // Make it so that nodes are at a certain height > 0
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
      Vector position = mobility->GetPosition ();
      position.z = 1.2;
      mobility->SetPosition (position);
    }

  // Create the LoraNetDevices of the end devices
  uint8_t nwkId = 54;
  uint32_t nwkAddr = 1864;
  Ptr<LoraDeviceAddressGenerator> addrGen =
      CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

  // Create the LoraNetDevices of the end devices
  macHelper.SetAddressGenerator (addrGen);
  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);
  helper.Install (phyHelper, macHelper, endDevices);

  // Now end devices are connected to the channel

  // Connect trace sources
  for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
    {
      Ptr<Node> node = *j;
      Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
      Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
    }

  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/


  
  PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
  appHelper.SetPeriod (Seconds (appPeriod));
  appHelper.SetPacketSize (packetSize);
  ApplicationContainer appContainer = appHelper.Install (endDevices);

  appContainer.Start (Seconds (0));
  appContainer.Stop (Seconds (simulationTime));

  /**************************
   *  Create Network Server  *
   ***************************/

  // Create the NS node
  NodeContainer networkServer;
  networkServer.Create (1);

  // Create a NS for the network
  nsHelper.SetEndDevices (endDevices);
  nsHelper.SetGateways (gateways);
  nsHelper.Install (networkServer);

  //Create a forwarder for each gateway
  forHelper.Install (gateways);

  // Install trace sources
  for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End(); node++)
  {
    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "ReceivedPacket", MakeCallback (OnPacketReceptionCallback));

    if (logProfile >= 1 ){
      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
          "LostPacketBecauseInterference", MakeCallback (OnPacketInterferenceCallback));

      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
          "LostPacketBecauseUnderSensitivity", MakeCallback (OnPacketUnderSensitivityCallback));
    }
  }

  // Install trace sources
  for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++)
  {
    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "StartSending", MakeCallback (OnTransmissionCallback));

      if (logProfile >= 2 ){
      (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
          "CannotSendBecauseDutyCycle", MakeCallback (OnPacketDutyCycle));
    }
  }

  //macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (Seconds(simulationTime+appPeriod));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  /////////////////////////////
  // Print results to stdout //
  /////////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");
  
  
  if(logProfile == 1){
    for (int i = 0; i < 6; i++)
    {
      std::cout << packetsSent.at(i) << " " << packetsReceived.at(i) << " " << packetsInterference.at(i) << " " << packetsUnderSensitivity.at(i) << " " ;
    }

  }
  else if(logProfile == 2){
    for (int i = 0; i < 6; i++)
    {
      std::cout << packetsSent.at(i) << " " << packetsReceived.at(i) << " " << packetsInterference.at(i) << " " << packetsUnderSensitivity.at(i) << " " << packetDutyCycle.at(i) <<" " ;
    }
  }
  else{
    for (int i = 0; i < 6; i++)
    {
      std::cout << packetsSent.at(i) << " " << packetsReceived.at(i) << " " ;
    }
  }
  std::cout << std::endl;
  return 0;
}

 
 