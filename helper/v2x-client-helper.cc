#include "ns3/application-container.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/object-factory.h"
#include "ns3/node-container.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/uinteger.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/names.h"

#include "v2x-client-helper.h"
#include "ns3/v2x-client.h"

NS_LOG_COMPONENT_DEFINE ("V2xClientHelper");

namespace ns3 {

V2xClientHelper::V2xClientHelper (Address address, bool withCAM)
{
  m_factory.SetTypeId ("ns3::V2xClient");
  m_factory.Set ("RemoteAddress", AddressValue (address));
  m_factory.Set ("WithCAM", BooleanValue (withCAM));
}

void
V2xClientHelper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
V2xClientHelper::Install (Ptr<Node> node) const
{
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
V2xClientHelper::Install (std::string nodeName) const
{
  Ptr<Node> node = Names::Find<Node> (nodeName);
  return ApplicationContainer (InstallPriv (node));
}

ApplicationContainer
V2xClientHelper::Install (NodeContainer c) const
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      apps.Add (InstallPriv (*i));
    }

  return apps;
}

Ptr<Application>
V2xClientHelper::InstallPriv (Ptr<Node> node) const
{
/*
  // Create LocationTable
  Ptr<LocationTable> locationTable = CreateObject<LocationTable> ();
  // Aggregate the LocationTable to the node
  node->AggregateObject (locationTable);
*/
  // Create Application
  Ptr<Application> app = m_factory.Create<Application> ();
  // Add application to the node
  node->AddApplication (app);

  return app;
}


} // namespace ns3
