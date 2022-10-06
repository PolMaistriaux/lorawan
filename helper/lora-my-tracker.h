/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 University of Padova
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
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

#ifndef LORA_MY_TRACKER_H
#define LORA_MY_TRACKER_H

#include "ns3/packet.h"
#include "ns3/nstime.h"
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
#include "ns3/forwarder-helper.h"

#include <map>
#include <string>

namespace ns3 {
namespace lorawan {

enum MyTracker {
  LostPacketBecauseInterference,
  LostPacketBecauseNoMoreReceivers,
  ReceivedPacketPHY,
  LostPacketBecauseWrongFrequency,
  LostPacketBecauseWrongSpreadingFactor,
  PhyRxBegin,
  NoReceptionBecauseTransmitting,
  LostPacketBecauseUnderSensitivity,
  PhyRxEnd,
  ReceivedPacketMAC,
  RandomSentMessage,
  PostponeTransmission,
  CannotSendBecauseDutyCycle,
  StartSending,
  SentNewPacket,
  ReceivedPacketNetwork,
  NewReceivedPacket
  
};

class LoRaMyTracker
{
public:
  LoRaMyTracker ();
  ~LoRaMyTracker ();

  void SetTrackerList (std::vector<MyTracker> &trackerList);
  void SetupTrackerNetworkServer (NodeContainer networkServer);
  void SetupTrackerEndDevices (NodeContainer endDevices);
  void SetupTrackerGateways (NodeContainer gateways);
  void Setup (NodeContainer networkServer, NodeContainer gateways, NodeContainer endDevices);

  void PrintTracker (void);

  void OnPacketSentApp (Ptr<Packet const> packet);
  int GetPacketSentApp (void);

  void OnPacketPostpone (Ptr<Packet const> packet);
  int GetPacketPostpone (void);

  void OnPacketDutyCycle (Ptr<Packet const> packet);
  int GetPacketDutyCycle (void);

  void OnPacketSentMAC (Ptr<Packet const> packet);
  int GetPacketSentMAC (void);

  void OnPacketSentPHY (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketSentPHY (void);

  void OnPacketWrongFreq (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketWrongFreq (void);

  void OnPacketWrongSF (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketWrongSF (void);

  void OnPacketBegRec (Ptr<Packet const> packet);
  int GetPacketBegRec (void);

  void OnPacketIsInTxCallback (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketIsInTxCallback (void);

  void OnPacketUnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketUnderSensitivityCallback (void);

  void OnPacketNoMoreDemod (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketNoMoreDemod (void);

  void OnPacketEndRec(Ptr<Packet const> packet);
  int GetPacketEndRec (void);

  void OnPacketInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketInterferenceCallback (void);

  void OnPacketRecPHY (Ptr<Packet const> packet, uint32_t systemId);
  int GetPacketRecPHY (void);

  void OnPacketRecMAC (Ptr<Packet const> packet);
  int GetPacketRecMAC (void);

  void OnPacketRecNetwork (Ptr<Packet const> packet);
  int GetPacketRecNetwork (void);

  void OnPacketRecNewNetwork (Ptr<Packet const> packet);
  int GetPacketRecNewNetwork (void);



private:
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
  int packetsRecNetwork = 0;
  int packetsRecNewNetwork = 0;
  
  std::vector<MyTracker> toTrack;
};
}
}
#endif
