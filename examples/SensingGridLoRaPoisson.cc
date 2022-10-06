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
#include "ns3/lora-my-tracker.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/sensing-field-position-allocator.h"
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
int sensing_radius = 1500;
int packetSize = 50;
double range = 5000;
double fieldSizeX = 10000;
double fieldSizeY = 10000;
double simulationTime = 3600;
double lambda = 2;
int logPositions = 1;
int maxReceptionPaths = 8;

// Channel model
bool realisticChannelModel = false;

uint8_t
ComputeSF (Ptr<Node> endDevice, NodeContainer gateways, Ptr<LoraChannel> channel) ;

void
ExportPositions (NodeContainer endDevices, NodeContainer gateways, Ptr<LoraChannel> channel, std::string nameEndDevices , std::string nameGateways , std::string Parameters);

int main (int argc, char *argv[])
{
  std::string interferenceMatrix = "aloha";

  CommandLine cmd;
  cmd.AddValue ("sensing_radius", "Sensing radius of sensor nodes", sensing_radius);
  cmd.AddValue ("simulationTime", "Simulation Time", simulationTime);
  cmd.AddValue ("fieldSizeX", "Field size in x", fieldSizeX);
  cmd.AddValue ("fieldSizeY", "Field size in y", fieldSizeY);
  cmd.AddValue ("range", "range of the gateway", range);
  cmd.AddValue ("lambda", "Mean number of packets per Hour", lambda);
  cmd.AddValue ("logPositions", "Only track sent and received packet", logPositions);
  cmd.AddValue ("packetSize", "Payload size if the packets", packetSize);
  cmd.AddValue ("maxReceptionPaths", "Number of reception paths", maxReceptionPaths);
  cmd.Parse (argc, argv);

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

  Ptr<RectangleFieldSensingPositionAllocator> positionAllocatorEndDevices =CreateObject<RectangleFieldSensingPositionAllocator>(fieldSizeX,fieldSizeY,sensing_radius, 5);
  int nEndDevices = positionAllocatorEndDevices->GetN();
  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAllocatorEndDevices);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  endDevices.Create (nEndDevices);
  mobility.Install (endDevices);

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
  //helper.EnablePacketTracking (); // Output filename

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

  Ptr<RectangleFieldGatewayPositionAllocator> positionAllocatorGateways =CreateObject<RectangleFieldGatewayPositionAllocator>(fieldSizeX,fieldSizeY,range, 15);
  int nGateways = positionAllocatorGateways->GetN();
  mobility.SetPositionAllocator (positionAllocatorGateways);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  gateways.Create (nGateways);
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
  appHelper.SetLambda (lambda);
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
  
  // Tracker installation
  
  LoRaMyTracker packetTracker = LoRaMyTracker();
  std::vector<MyTracker> toTrack = {RandomSentMessage, PostponeTransmission, StartSending,LostPacketBecauseNoMoreReceivers, LostPacketBecauseInterference,ReceivedPacketPHY,  NewReceivedPacket};
  packetTracker.SetTrackerList(toTrack);
  packetTracker.SetupTrackerEndDevices(endDevices);
  packetTracker.SetupTrackerGateways(gateways);
  packetTracker.SetupTrackerNetworkServer(networkServer);
  std::vector<int> SFdistribution = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
  
  /////////////////////////////
  // Print results to stdout //
  /////////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");
  if (logPositions ==1){
    ExportPositions (endDevices, gateways, channel, "Results_Simu/endDevicesPositions.txt" ,"Results_Simu/gatewaysPositions.txt" , "Results_Simu/parameters.txt");
  }

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (Seconds(simulationTime) +Seconds(10));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();
  
  std::cout << nEndDevices << " " << nGateways << " ";
  packetTracker.PrintTracker ();
  std::cout << std::endl;

  return 0;

}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  ADDITIONNAL FUNCTIONS                                         //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t
ComputeSF (Ptr<Node> endDevice, NodeContainer gateways, Ptr<LoraChannel> channel)
{
  Ptr<MobilityModel> position = endDevice->GetObject<MobilityModel> ();
  Ptr<NetDevice> netDevice = endDevice->GetDevice (0);
  Ptr<LoraNetDevice> loraNetDevice = netDevice->GetObject<LoraNetDevice> ();
  Ptr<EndDeviceLorawanMac> mac =
      loraNetDevice->GetMac ()->GetObject<EndDeviceLorawanMac> ();

  // Try computing the distance from each gateway and find the best one
  Ptr<Node> bestGateway = gateways.Get (0);
  Ptr<MobilityModel> bestGatewayPosition = bestGateway->GetObject<MobilityModel> ();

  // Assume devices transmit at 14 dBm
  double highestRxPower = channel->GetRxPower (14, position, bestGatewayPosition);

  for (NodeContainer::Iterator currentGw = gateways.Begin () + 1; currentGw != gateways.End ();
        ++currentGw)
    {
      // Compute the power received from the current gateway
      Ptr<Node> curr = *currentGw;
      Ptr<MobilityModel> currPosition = curr->GetObject<MobilityModel> ();
      double currentRxPower = channel->GetRxPower (14, position, currPosition); // dBm

      if (currentRxPower > highestRxPower)
        {
          bestGateway = curr;
          bestGatewayPosition = curr->GetObject<MobilityModel> ();
          highestRxPower = currentRxPower;
        }
    }

  double rxPower = highestRxPower;
  Ptr<EndDeviceLoraPhy> edPhy = loraNetDevice->GetPhy ()->GetObject<EndDeviceLoraPhy> ();
  const double *edSensitivity = edPhy->sensitivity;

  if (rxPower > *edSensitivity)
      return mac->GetSfFromDataRate (5);
  else if (rxPower > *(edSensitivity + 1))
      return mac->GetSfFromDataRate (4);
  else if (rxPower > *(edSensitivity + 2))
      return mac->GetSfFromDataRate (3);
  else if (rxPower > *(edSensitivity + 3))
      return mac->GetSfFromDataRate (2);
  else if (rxPower > *(edSensitivity + 4))
      return mac->GetSfFromDataRate (1);
  else if (rxPower > *(edSensitivity + 5))
      return mac->GetSfFromDataRate (0);
  else // Device is out of range. Assign SF12.
      return 13;
}



void
ExportPositions (NodeContainer endDevices, NodeContainer gateways, Ptr<LoraChannel> channel, std::string nameEndDevices , std::string nameGateways , std::string Parameters)
{
// iterate our nodes and print their position.
std::ofstream outputFile;
// Delete contents of the file as it is opened
outputFile.open (nameEndDevices, std::ofstream::out | std::ofstream::trunc);
for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
  {
    Ptr<Node> object = *j;
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    NS_ASSERT (position != 0);
    Vector pos = position->GetPosition ();
    //Ptr<EndDeviceLorawanMac> nodeMac = (*j)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac () ->GetObject<EndDeviceLorawanMac> ();
    uint8_t sf = ComputeSF (object, gateways, channel);
    outputFile << pos.x << " " << pos.y << " " << pos.z << " " << +sf << std::endl;
  }
outputFile.close ();


outputFile.open (nameGateways, std::ofstream::out | std::ofstream::trunc);
for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End (); ++j)
{
  Ptr<Node> object = *j;
  Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
  NS_ASSERT (position != 0);
  Vector pos = position->GetPosition ();
  outputFile << pos.x << " " << pos.y << " " << pos.z << std::endl;
}
outputFile.close ();

outputFile.open (Parameters, std::ofstream::out | std::ofstream::trunc);
outputFile << range << " " << sensing_radius << " " << fieldSizeX << " " << fieldSizeY << " " << std::endl;
outputFile.close ();
}       