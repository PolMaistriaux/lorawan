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

#include "ns3/random-traffic-sender-helper.h"
#include "ns3/random-variable-stream.h"
#include "ns3/random-traffic-sender.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/simulator.h"
#include "ns3/log.h"

namespace ns3 {
namespace lorawan {

NS_LOG_COMPONENT_DEFINE ("RandomTrafficSenderHelper");

RandomTrafficSenderHelper::RandomTrafficSenderHelper ()
{
  m_factory.SetTypeId ("ns3::RandomTrafficSender");

  m_pktSize = 10;
  m_pktSizeRV = 0;
}

RandomTrafficSenderHelper::~RandomTrafficSenderHelper ()
{
}

void
RandomTrafficSenderHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}


ApplicationContainer
RandomTrafficSenderHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
RandomTrafficSenderHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
RandomTrafficSenderHelper::InstallPriv (Ptr<Node> node) const
{
  NS_LOG_FUNCTION (this << node);
  Ptr<RandomTrafficSender> app = m_factory.Create<RandomTrafficSender> ();
  app->SetLambda (helper_lambda);
  NS_LOG_DEBUG ("Created an application with lambda = " <<
                helper_lambda);
  app->SetPacketSize (m_pktSize);
  if (m_pktSizeRV)
    {
      app->SetPacketSizeRandomVariable (m_pktSizeRV);
    }
  app->SetNode (node);
  node->AddApplication (app);
  return app;
}

void
RandomTrafficSenderHelper::SetLambda (double lambda)
{
  helper_lambda = lambda;
}

void
RandomTrafficSenderHelper::SetPacketSizeRandomVariable (Ptr <RandomVariableStream> rv)
{
  m_pktSizeRV = rv;
}

void
RandomTrafficSenderHelper::SetPacketSize (uint8_t size)
{
  m_pktSize = size;
}

}
} // namespace ns3
