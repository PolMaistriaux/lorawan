/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 University of Padova
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

#include "ns3/random-traffic-sender.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/lora-net-device.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("RandomTrafficSender");

NS_OBJECT_ENSURE_REGISTERED (RandomTrafficSender);

TypeId
RandomTrafficSender::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RandomTrafficSender")
    .SetParent<Application> ()
    .AddConstructor<RandomTrafficSender> ()
    .SetGroupName ("lorawan")
    .AddAttribute ("Lambda", "Mean arrival rate of events (per hour)",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&RandomTrafficSender::GetLambda,
                                     &RandomTrafficSender::SetLambda),
                   MakeTimeChecker ())
    .AddTraceSource ("RandomSentMessage",
                     "Trace source indicating a packet "
                     "went from Periodic sender to MAC layer",
                     MakeTraceSourceAccessor (&RandomTrafficSender::m_periodPktSentToMac),
                     "ns3::Packet::TracedCallback");
  // .AddAttribute ("PacketSizeRandomVariable", "The random variable that determines the shape of the packet size, in bytes",
  //                StringValue ("ns3::UniformRandomVariable[Min=0,Max=10]"),
  //                MakePointerAccessor (&PeriodicSender::m_pktSizeRV),
  //                MakePointerChecker <RandomVariableStream>());
  return tid;
}

RandomTrafficSender::RandomTrafficSender ()
  : m_lambda (3),
  m_basePktSize (10),
  m_pktSizeRV (0)

{
  NS_LOG_FUNCTION_NOARGS ();
}

RandomTrafficSender::~RandomTrafficSender ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
RandomTrafficSender::SetLambda (double lambda)
{
  NS_LOG_FUNCTION (this << lambda);
  m_lambda = lambda;
}

double
RandomTrafficSender::GetLambda (void) const
{
  NS_LOG_FUNCTION (this);
  return m_lambda;
}

void
RandomTrafficSender::SetPacketSize (uint8_t size)
{
  m_basePktSize = size;
}

void
RandomTrafficSender::SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv)
{
  m_pktSizeRV = rv;
}


void
RandomTrafficSender::SendPacket (void)
{
  NS_LOG_FUNCTION (this);

  // Create and send a new packet
  Ptr<Packet> packet;
  if (m_pktSizeRV)
    {
      int randomsize = m_pktSizeRV->GetInteger ();
      packet = Create<Packet> (m_basePktSize + randomsize);
    }
  else
    {
      packet = Create<Packet> (m_basePktSize);
    }
  m_mac->Send (packet);
  m_periodPktSentToMac(packet);
  Time t_poisson_arrival = Hours (exp->GetValue());
  // Schedule the next SendPacket event
  m_sendEvent = Simulator::Schedule (t_poisson_arrival, &RandomTrafficSender::SendPacket,
                                     this);

  NS_LOG_DEBUG ("Sent a packet of size " << packet->GetSize ());
}

void
RandomTrafficSender::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  // Make sure we have a MAC layer
  if (m_mac == 0)
    {
      // Assumes there's only one device
      Ptr<LoraNetDevice> loraNetDevice = m_node->GetDevice (0)->GetObject<LoraNetDevice> ();

      m_mac = loraNetDevice->GetMac ();
      NS_ASSERT (m_mac != 0);
    }
  
  double inter_event_interval= 1/ m_lambda;
  exp = CreateObject<ExponentialRandomVariable> ();
  exp->SetAttribute ("Mean", DoubleValue (inter_event_interval));
  Time t_poisson_arrival = Hours (exp->GetValue());

  // Schedule the next SendPacket event
  Simulator::Cancel (m_sendEvent);
  NS_LOG_DEBUG ("Starting up application with a first event with a " <<
                t_poisson_arrival.GetSeconds () << " seconds delay");
  m_sendEvent = Simulator::Schedule (t_poisson_arrival,
                                     &RandomTrafficSender::SendPacket, this);
  NS_LOG_DEBUG ("Event Id: " << m_sendEvent.GetUid ());
}

void
RandomTrafficSender::StopApplication (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Simulator::Cancel (m_sendEvent);
}

}
}
