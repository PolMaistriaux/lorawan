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
#include "ns3/random-traffic-sender-helper.h"
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


// Network settings
int sensing_radius = 100;
int nGateways = 1;
int packetSize = 50;
double radius = 1200;
double simulationTime = 3600;
double appPeriod = 50;
int logProfile = 1;
int maxReceptionPaths = 8;

// Channel model
bool realisticChannelModel = false;

int packetsSentApp =0;
int packetsPostpone =0;
int packetsDutyCycle =0;
int packetsSentMAC =0;
int packetsSentPhy =0;
int packetsWrongFreq =0;
int packetsWrongSF =0;
int packetsInterference =0;
int packetsBegRec =0;
int packetsUnderSensitivity =0;
int packetsIsInTx =0;
int packetsNoMoreDemod =0;
int packetsEndRec =0;
int packetsRecPHY =0;
int packetsRecMAC =0;

void
OnPacketSentApp (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsSentApp++;
}

void
OnPacketPostpone (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsPostpone++;
}

void
OnPacketDutyCycle (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsDutyCycle++;
}

void
OnPacketSentMAC (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsSentMAC++;
}

void
OnPacketSentPHY (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsSentPhy++;
}

void
OnPacketWrongFreq (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsWrongFreq++;
}

void
OnPacketWrongSF (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsWrongSF++;
}

void
OnPacketBegRec (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsBegRec++;
}

void
OnPacketIsInTxCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsIsInTx++;
}

void
OnPacketUnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsUnderSensitivity++;
}

void
OnPacketNoMoreDemod (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsNoMoreDemod++;
}

void
OnPacketEndRec(Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsEndRec++;
}

void
OnPacketInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsInterference++;
}



void
OnPacketRecPHY (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsRecPHY++;
}


void
OnPacketRecMAC (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsRecMAC++;
}

int main (int argc, char *argv[])
{
  std::string interferenceMatrix = "aloha";
  
  CommandLine cmd;
  cmd.AddValue ("sensing_radius", "Sensing radius of sensor nodes", sensing_radius);
  cmd.AddValue ("simulationTime", "Simulation Time", simulationTime);
  cmd.AddValue ("radius", "Radius of the deployment", radius);
  cmd.AddValue ("appPeriod", "Time in seconds between two packet at a node", appPeriod);
  cmd.AddValue ("logProfile", "Only track sent and received packet", logProfile);
  cmd.AddValue ("packetSize", "Payload size if the packets", packetSize);
  cmd.AddValue ("maxReceptionPaths", "Number of reception paths", maxReceptionPaths);
  cmd.Parse (argc, argv);

  //simulationTime = 10*appPeriod;
  // Set up logging
  //LogComponentEnable ("AlohaThroughput", LOG_LEVEL_ALL);
  //LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ERROR);
  //LogComponentEnable ( "SimpleEndDeviceLoraPhy", LOG_LEVEL_FUNCTION);
  //LogComponentEnable ( "SimpleGatewayLoraPhy", LOG_LEVEL_FUNCTION);
  //LogComponentEnable ( "LoraPhyHelper", LOG_LEVEL_FUNCTION);
  //LogComponentEnable ( "GatewayLoraPhy", LOG_LEVEL_FUNCTION);
  //LogComponentEnable ( "LorawanMacHelper", LOG_LEVEL_FUNCTION);
  //LogComponentEnable ( "EndDeviceLorawanMac", LOG_LEVEL_FUNCTION);

  // Make all devices use SF7 (i.e., DR0)
  //Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (0));

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

  NodeContainer endDevices;
  
  SensingGridPositionAllocator positionAllocator = SensingGridPositionAllocator();
  positionAllocator.SetRadius(radius);
  positionAllocator.SetSensingRadius(sensing_radius);
  positionAllocator.SetZ(5.0);
  positionAllocator.Initialize();


  // each object will be attached a static position.
  // i.e., once set by the "position allocator", the
  // position will never change.
  ObjectFactory m_mobility;
  m_mobility.SetTypeId ("ns3::ConstantPositionMobilityModel");

  while(positionAllocator.HasNext()){
    Ptr<Object> object = CreateObject<Node> ();
    Ptr<MobilityModel> model = m_mobility.Create ()->GetObject<MobilityModel> ();
    object->AggregateObject (model);

    Vector position = positionAllocator.GetNext ();
    model->SetPosition (position);
    endDevices.Add(object->GetObject<Node> ());
  }
  int n = positionAllocator.GetN();

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
  phyHelper.SetSimulateDownLink (false);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();
  macHelper.SetRegion (LorawanMacHelper::EU);
  macHelper.SetDutyCycleCustom(0.01);
  macHelper.SetMacMaxReceptionPaths(maxReceptionPaths);

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
  /*********************
   *  Create Gateways  *
   *********************/

  // Create the gateway nodes (allocate them uniformely on the disc)
  NodeContainer gateways;
  gateways.Create (nGateways);

    // Mobility
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  // Make it so that nodes are at a certain height > 0
  allocator->Add (Vector (0.0, 0.0, 15.0));
  mobility.SetPositionAllocator (allocator);
  mobility.Install (gateways);

  // Create a netdevice for each gateway
  phyHelper.SetDeviceType (LoraPhyHelper::GW);
  phyHelper.SetMaxReceptionPaths(8);
  macHelper.SetDeviceType (LorawanMacHelper::GW);
  helper.Install (phyHelper, macHelper, gateways);

  NS_LOG_DEBUG ("Completed configuration");

  /*********************************************
   *  Install applications on the end devices  *
   *********************************************/
  
  RandomTrafficSenderHelper appHelper = RandomTrafficSenderHelper ();
  appHelper.SetLambda (appPeriod);
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
          "LostPacketBecauseInterference", MakeCallback (OnPacketInterferenceCallback));

    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "LostPacketBecauseNoMoreReceivers", MakeCallback (OnPacketNoMoreDemod));
  
    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "ReceivedPacket", MakeCallback (OnPacketRecPHY));

      if (logProfile >0){
        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "LostPacketBecauseWrongFrequency", MakeCallback (OnPacketWrongFreq));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "LostPacketBecauseWrongSpreadingFactor", MakeCallback (OnPacketWrongSF));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "PhyRxBegin", MakeCallback (OnPacketBegRec));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "NoReceptionBecauseTransmitting", MakeCallback (OnPacketIsInTxCallback));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
          "LostPacketBecauseUnderSensitivity", MakeCallback (OnPacketUnderSensitivityCallback));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "PhyRxEnd", MakeCallback (OnPacketEndRec));

        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
        "ReceivedPacket", MakeCallback (OnPacketRecMAC));
      }
    }
 

  // Install trace sources
  for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++)
  {

    (*node)->GetApplication (0)->GetObject<RandomTrafficSender> ()->TraceConnectWithoutContext (
      "RandomSentMessage", MakeCallback (OnPacketSentApp));

    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
          "PostponeTransmission", MakeCallback (OnPacketPostpone));

    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
          "CannotSendBecauseDutyCycle", MakeCallback (OnPacketDutyCycle));

    (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
        "StartSending", MakeCallback (OnPacketSentPHY));

    if (logProfile >0){
        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext (
        "SentNewPacket", MakeCallback (OnPacketSentMAC));
    }

  }



  std::vector<int> SFdistribution = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
  
  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (Seconds(simulationTime) +Seconds(10));
  //Simulator::Stop (Seconds(simulationTime+appPeriod));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  /////////////////////////////
  // Print results to stdout //
  /////////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");
  /*
  // iterate our nodes and print their position.
  std::ofstream outputFile;
  // Delete contents of the file as it is opened
  outputFile.open ("Results_Simu/positions.txt", std::ofstream::out | std::ofstream::trunc);
  for (NodeContainer::Iterator j = nodes.Begin (); j != nodes.End (); ++j)
    {
      Ptr<Node> object = *j;
      Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
      NS_ASSERT (position != 0);
      Vector pos = position->GetPosition ();
      outputFile << pos.x << " " << pos.y << " " << pos.z << std::endl;
    }
    outputFile.close ();*/


  if(logProfile == 0){
    std::cout << n << " " << packetsSentApp << " "  << (packetsPostpone+packetsDutyCycle) << " " << packetsSentPhy << " " << packetsNoMoreDemod << " " << packetsInterference  << " " << packetsRecPHY << " ";
  }
  else if(logProfile == 1){
    std::cout << n << " " << packetsSentApp << " " << packetsPostpone << " " << packetsDutyCycle << " " << packetsSentMAC << " " << packetsSentPhy << " " << packetsWrongFreq << " " << packetsWrongSF << " " << packetsBegRec << " " << packetsIsInTx << " " << packetsUnderSensitivity << " " << packetsNoMoreDemod << " " << packetsEndRec << " " << packetsInterference << " " << packetsRecPHY << " " << packetsRecMAC << " ";
  }
  std::cout << std::endl;
  return 0;

}
