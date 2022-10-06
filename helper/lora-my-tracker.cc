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

#include "lora-my-tracker.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include <iostream>
#include <fstream>

namespace ns3 {
namespace lorawan {
NS_LOG_COMPONENT_DEFINE ("LoRaMyTracker");

LoRaMyTracker::LoRaMyTracker ()
{
  NS_LOG_FUNCTION (this);
}

LoRaMyTracker::~LoRaMyTracker ()
{
  NS_LOG_FUNCTION (this);
}

void 
LoRaMyTracker::SetTrackerList (std::vector<MyTracker> &trackerList)
{
  toTrack.clear();
  toTrack = trackerList;
}




void 
LoRaMyTracker::Setup (NodeContainer networkServer, NodeContainer gateways, NodeContainer endDevices)
{
  SetupTrackerGateways (gateways);
  SetupTrackerEndDevices (endDevices);
  SetupTrackerNetworkServer (networkServer);
}

void
LoRaMyTracker::SetupTrackerGateways (NodeContainer gateways)
{
  // Install trace sources
  for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End(); node++)
  {
    for (std::vector<MyTracker>::iterator tracking = toTrack.begin(); tracking != toTrack.end(); tracking++){
      switch (*tracking){
        case LostPacketBecauseInterference:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("LostPacketBecauseInterference", MakeCallback (&LoRaMyTracker::OnPacketInterferenceCallback, this));
          break;

        case LostPacketBecauseNoMoreReceivers:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("LostPacketBecauseNoMoreReceivers", MakeCallback (&LoRaMyTracker::OnPacketNoMoreDemod, this));
          break;
      
        case ReceivedPacketPHY:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&LoRaMyTracker::OnPacketRecPHY, this));
          break;

        case LostPacketBecauseWrongFrequency:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("LostPacketBecauseWrongFrequency", MakeCallback (&LoRaMyTracker::OnPacketWrongFreq, this));
          break;

        case LostPacketBecauseWrongSpreadingFactor:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("LostPacketBecauseWrongSpreadingFactor", MakeCallback (&LoRaMyTracker::OnPacketWrongSF, this));
          break;

        case PhyRxBegin:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("PhyRxBegin", MakeCallback (&LoRaMyTracker::OnPacketBegRec, this));
          break;

        case NoReceptionBecauseTransmitting:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("NoReceptionBecauseTransmitting", MakeCallback (&LoRaMyTracker::OnPacketIsInTxCallback, this));
          break;

        case LostPacketBecauseUnderSensitivity:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (  "LostPacketBecauseUnderSensitivity", MakeCallback (&LoRaMyTracker::OnPacketUnderSensitivityCallback, this));
          break;

        case PhyRxEnd:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&LoRaMyTracker::OnPacketEndRec, this));
          break;

        case ReceivedPacketMAC:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&LoRaMyTracker::OnPacketRecMAC, this));
          break;
        
        default:
          break;
      }
    }
  }
}

void
LoRaMyTracker::SetupTrackerEndDevices (NodeContainer endDevices)
{
  // Install trace sources
  for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++)
  {
    for (std::vector<MyTracker>::iterator tracking = toTrack.begin(); tracking != toTrack.end(); tracking++){
      switch (*tracking){
        case RandomSentMessage:
          (*node)->GetApplication (0)->GetObject<RandomTrafficSender> ()->TraceConnectWithoutContext ("RandomSentMessage", MakeCallback (&LoRaMyTracker::OnPacketSentApp, this));
          break;

        case PostponeTransmission:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext ("PostponeTransmission", MakeCallback (&LoRaMyTracker::OnPacketPostpone, this));
          break;

        case CannotSendBecauseDutyCycle:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext ("CannotSendBecauseDutyCycle", MakeCallback (&LoRaMyTracker::OnPacketDutyCycle, this));
          break;

        case StartSending:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext ("StartSending", MakeCallback (&LoRaMyTracker::OnPacketSentPHY, this));
          break;

        case SentNewPacket:
          (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac ()->TraceConnectWithoutContext ( "SentNewPacket", MakeCallback (&LoRaMyTracker::OnPacketSentMAC, this));
          break;
        
        default:
          break;
      }
    }
  }
}

void
LoRaMyTracker::SetupTrackerNetworkServer (NodeContainer networkServer)
{
  // Install trace sources
  for (NodeContainer::Iterator node = networkServer.Begin (); node != networkServer.End(); node++)
  {
    for (std::vector<MyTracker>::iterator tracking = toTrack.begin(); tracking != toTrack.end(); tracking++){
      switch (*tracking){
        case ReceivedPacketNetwork:
          (*node)->GetApplication (0)->GetObject<NetworkServer> ()->TraceConnectWithoutContext ("ReceivedPacket", MakeCallback (&LoRaMyTracker::OnPacketRecNetwork, this));
          break;

        case NewReceivedPacket:
          (*node)->GetApplication (0)->GetObject<NetworkServer> ()-> GetNetworkStatus()->TraceConnectWithoutContext ("NewReceivedPacket", MakeCallback (&LoRaMyTracker::OnPacketRecNewNetwork, this));
          break;
        
        default:
          break;
      }
    }
  }
}

void
LoRaMyTracker::PrintTracker (void)
{
  for (std::vector<MyTracker>::iterator tracking = toTrack.begin(); tracking != toTrack.end(); tracking++){

    switch (*tracking){
        case LostPacketBecauseInterference :
          std::cout << packetsInterference << " ";
          break;
        case LostPacketBecauseNoMoreReceivers :
          std::cout << packetsNoMoreDemod << " ";
          break;
        case ReceivedPacketPHY :
          std::cout << packetsRecPHY << " ";
          break;
        case LostPacketBecauseWrongFrequency :
          std::cout << packetsWrongFreq << " ";
          break;
        case LostPacketBecauseWrongSpreadingFactor :
          std::cout << packetsWrongSF << " ";
          break;
        case PhyRxBegin :
          std::cout << packetsBegRec << " ";
          break;
        case NoReceptionBecauseTransmitting :
          std::cout << packetsIsInTx << " ";
          break;
        case LostPacketBecauseUnderSensitivity :
          std::cout << packetsUnderSensitivity << " ";
          break;
        case PhyRxEnd :
          std::cout << packetsEndRec << " ";
          break;
        case ReceivedPacketMAC :
          std::cout << packetsRecMAC << " ";
          break;
        case RandomSentMessage :
          std::cout << packetsSentApp << " ";
          break;
        case PostponeTransmission :
          std::cout << packetsPostpone << " ";
          break;
        case CannotSendBecauseDutyCycle :
          std::cout << packetsDutyCycle << " ";
          break;
        case StartSending :
          std::cout << packetsSentPhy << " ";
          break;
        case SentNewPacket :
          std::cout << packetsSentMAC << " ";
          break;
        case ReceivedPacketNetwork :
          std::cout << packetsRecNetwork << " ";
          break;
        case NewReceivedPacket :
          std::cout << packetsRecNewNetwork << " ";
          break;
        default:
         std::cout <<"Unknown tracker" <<  std::endl;
          break;
         
    }
  }

}
  
//////////////////////////////////////
//      CallBack Functions          //
//////////////////////////////////////

void
LoRaMyTracker::OnPacketSentApp (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsSentApp++;
}

int
LoRaMyTracker::GetPacketSentApp (void)
{
  return packetsSentApp;
}

void
LoRaMyTracker::OnPacketPostpone (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsPostpone++;
}

int
LoRaMyTracker::GetPacketPostpone (void)
{
  return packetsPostpone;
}

void
LoRaMyTracker::OnPacketDutyCycle (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsDutyCycle++;
}

int
LoRaMyTracker::GetPacketDutyCycle (void)
{
  return packetsDutyCycle;
}

void
LoRaMyTracker::OnPacketSentMAC (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsSentMAC++;
}

int
LoRaMyTracker::GetPacketSentMAC (void)
{
  return packetsSentMAC;
}

void
LoRaMyTracker::OnPacketSentPHY (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsSentPhy++;
}

int
LoRaMyTracker::GetPacketSentPHY (void)
{
  return packetsSentPhy;
}

void
LoRaMyTracker::OnPacketWrongFreq (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsWrongFreq++;
}

int
LoRaMyTracker::GetPacketWrongFreq (void)
{
  return packetsWrongFreq;
}

void
LoRaMyTracker::OnPacketWrongSF (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsWrongSF++;
}

int
LoRaMyTracker::GetPacketWrongSF (void)
{
  return packetsWrongSF;
}

void
LoRaMyTracker::OnPacketBegRec (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsBegRec++;
}

int
LoRaMyTracker::GetPacketBegRec (void)
{
  return packetsBegRec;
}

void
LoRaMyTracker::OnPacketIsInTxCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsIsInTx++;
}

int
LoRaMyTracker::GetPacketIsInTxCallback (void)
{
  return packetsIsInTx;
}

void
LoRaMyTracker::OnPacketUnderSensitivityCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsUnderSensitivity++;
}

int
LoRaMyTracker::GetPacketUnderSensitivityCallback (void)
{
  return packetsUnderSensitivity;
}

void
LoRaMyTracker::OnPacketNoMoreDemod (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsNoMoreDemod++;
}

int
LoRaMyTracker::GetPacketNoMoreDemod (void)
{
  return packetsNoMoreDemod;
}

void
LoRaMyTracker::OnPacketEndRec(Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsEndRec++;
}

int
LoRaMyTracker::GetPacketEndRec (void)
{
  return packetsEndRec;
}

void
LoRaMyTracker::OnPacketInterferenceCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsInterference++;
}

int
LoRaMyTracker::GetPacketInterferenceCallback (void)
{
  return packetsInterference;
}

void
LoRaMyTracker::OnPacketRecPHY (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  packetsRecPHY++;
}

int
LoRaMyTracker::GetPacketRecPHY (void)
{
  return packetsRecPHY;
}

void
LoRaMyTracker::OnPacketRecMAC (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsRecMAC++;
}

int
LoRaMyTracker::GetPacketRecMAC (void)
{
  return packetsRecMAC;
}

void
LoRaMyTracker::OnPacketRecNetwork (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsRecNetwork++;
}

int
LoRaMyTracker::GetPacketRecNetwork (void)
{
  return packetsRecNetwork;
}

void
LoRaMyTracker::OnPacketRecNewNetwork (Ptr<Packet const> packet)
{
  NS_LOG_FUNCTION (packet);
  packetsRecNewNetwork++;
}

int
LoRaMyTracker::GetPacketRecNewNetwork (void)
{
  return packetsRecNewNetwork;
}

}
}
