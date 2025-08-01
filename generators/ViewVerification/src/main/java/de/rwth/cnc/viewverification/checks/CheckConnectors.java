/* (c) https://github.com/MontiCore/monticore */
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
