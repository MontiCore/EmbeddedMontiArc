/**
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.cnc.viewverification.checks;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyMissingConnection;

public class CheckConnectors extends Checker {

  private List<InconsistencyMissingConnection> missingConnections;

  public CheckConnectors(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<InconsistencyMissingConnection> getMissingConnections() {
    return missingConnections;
  }

  /**
   * checks consistency of connectors (ignore components that are not part of
   * the architecture)
   *
   * @return true if arch chains of concrete connectors for all abstract
   * connectors from the view
   */
  @Override
  public boolean checkConsistency() {

    missingConnections = new ArrayList<InconsistencyMissingConnection>();

    Set<String> cmpsToCheck = new LinkedHashSet<String>();
    cmpsToCheck.addAll(view.getComponentNames());
    cmpsToCheck.retainAll(system.getComponentNames());

    for (Connection absConn : view.getConnections()) {
      if (system.getComponentNames().contains(absConn.getSender()) && system.getComponentNames().contains(absConn.getReceiver())) {
        checkAbstractConnector(absConn);
      }
    }

    return missingConnections.isEmpty();
  }

  private void checkAbstractConnector(Connection absConn) {
    boolean connectorChainFound = false;

    if (absConn.isComponentToComponent()) {
      for (Port p : system.getComponent(absConn.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = absConn.getSender() + "." + p.getName();
        if (system.isConnected(senderPort, absConn.getReceiver())) {
          connectorChainFound = true;
        }
      }
    }
    else if (absConn.isPortToComponent()) {
      String senderPort = absConn.getSender() + "." + absConn.getSenderPort();
      if (system.isConnected(senderPort, absConn.getReceiver())) {
        connectorChainFound = true;
      }
    }
    else if (absConn.isComponentToPort()) {
      for (Port p : system.getComponent(absConn.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = absConn.getSender() + "." + p.getName();
        String receiverPort = absConn.getReceiver() + "." + absConn.getReceiverPort();
        if (system.isConnected(senderPort, receiverPort)) {
          connectorChainFound = true;
        }
      }
    }
    else if (absConn.isPortToPort()) {
      String senderPort = absConn.getSender() + "." + absConn.getSenderPort();
      String receiverPort = absConn.getReceiver() + "." + absConn.getReceiverPort();
      if (system.isConnected(senderPort, receiverPort)) {
        connectorChainFound = true;
      }
    }

    if (!connectorChainFound) {
      InconsistencyMissingConnection missingConnection = new InconsistencyMissingConnection(absConn.getSender(), absConn.getReceiver(), absConn.getSenderPort(), absConn.getReceiverPort());
      missingConnections.add(missingConnection);
    }

  }

}
