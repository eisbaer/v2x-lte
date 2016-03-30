#ifndef V2X_CLIENT_HELPER_H
#define V2X_CLIENT_HELPER_H

#include <stdint.h>
#include <string>
#include "ns3/object-factory.h"
#include "ns3/address.h"
#include "ns3/attribute.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/boolean.h"

#include "ns3/application-container.h"

namespace ns3 {

class V2xClientHelper
{
public:

  V2xClientHelper (Address address, bool withCAM);

  // Record an attribute to be set in each Application after it is is created
  void SetAttribute (std::string name, const AttributeValue &value);

  /**
   * Install an ns3::V2xClient on each node of the input container
   * configured with all the attributes set with SetAttribute.
   *
   * \param c NodeContainer of the set of nodes on which an V2xClient
   * will be installed.
   * \returns Container of Ptr to the applications installed.
   */
  ApplicationContainer Install (NodeContainer c) const;

  /**
   * Install an ns3::V2xClient on the node configured with all the
   * attributes set with SetAttribute.
   *
   * \param node The node on which an V2xClient will be installed.
   * \returns Container of Ptr to the applications installed.
   */
  ApplicationContainer Install (Ptr<Node> node) const;

  /**
   * Install an ns3::V2xClient on the node configured with all the
   * attributes set with SetAttribute.
   *
   * \param nodeName The node on which an V2xClient will be installed.
   * \returns Container of Ptr to the applications installed.
   */
  ApplicationContainer Install (std::string nodeName) const;

private:
  /**
   * Install an ns3::V2xClient on the node configured with all the
   * attributes set with SetAttribute.
   *
   * \param node The node on which an V2xClient will be installed.
   * \returns Ptr to the application installed.
   */
  Ptr<Application> InstallPriv (Ptr<Node> node) const;

  ObjectFactory m_factory;

};

} // namespace ns3

#endif /* V2X_CLIENT_HELPER_H */
