/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.checks;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.inconsistency.InconsistencyMissingEffector;

public class CheckEffectors extends Checker {

  private List<InconsistencyMissingEffector> missingEffectors;

  public CheckEffectors(CnCArchitecture system, CnCView view) {
    this.system = system;
    this.view = view;
  }

  public List<InconsistencyMissingEffector> getMissingEffectors() {
    return missingEffectors;
  }

  /**
   * checks consistency of connectors (ignore components that are not part of
   * the architecture)
   *
   * @param archFilename
   * @param viewFilename
   * @return true if arch chains of concrete connectors for all abstract
   * connectors from the view
   */
  @Override
  public boolean checkConsistency() {

    missingEffectors = new ArrayList<InconsistencyMissingEffector>();

    // relevant components: (1) in view and (2) also in model
    Set<String> cmpsToCheck = new LinkedHashSet<String>();
    cmpsToCheck.addAll(view.getComponentNames());
    cmpsToCheck.retainAll(system.getComponentNames());

    for (Effector eff : view.getEffectors()) {
      if ((system.getComponentNames().contains(eff.getSender()) && system.getComponentNames().contains(eff.getReceiver()))) {
        checkEffector(eff);
      }
    }

    return missingEffectors.isEmpty();
  }

  private void checkEffector(Effector eff) {
    boolean connectorChainFound = false;

    if (eff.isComponentToComponent()) {
      Port possiblePort = view.getComponent(eff.getSender()).getPort(eff.getSenderPort());

      for (Port p : system.getComponent(eff.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = eff.getSender() + "." + p.getName();
        if (system.isEffected(senderPort, eff.getReceiver())) {
          if(possiblePort != null && possiblePort.isTextualAnonymous()) {
            if (!possiblePort.getName().equals(p.getName()))
              if (p.getDirection() == possiblePort.getDirection())
                connectorChainFound = true;
          }
          else
            connectorChainFound = true;
        }
      }
    }
    else if (eff.isPortToComponent()) {
      String senderPort = eff.getSender() + "." + eff.getSenderPort();
      if (system.isEffected(senderPort, eff.getReceiver())) {
        connectorChainFound = true;
      }
    }
    else if (eff.isComponentToPort()) {
      Port possiblePort = view.getComponent(eff.getSender()).getPort(eff.getSenderPort());

      for (Port p : system.getComponent(eff.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = eff.getSender() + "." + p.getName();
        String receiverPort = eff.getReceiver() + "." + eff.getReceiverPort();
        if (system.isEffected(senderPort, receiverPort)) {
          if(possiblePort != null && possiblePort.isTextualAnonymous()) {
            if (!possiblePort.getName().equals(p.getName()))
              if (p.getDirection() == possiblePort.getDirection())
                connectorChainFound = true;
          }
          else
            connectorChainFound = true;
        }
      }
    }
    else if (eff.isPortToPort()) {
      String senderPort = eff.getSender() + "." + eff.getSenderPort();
      String receiverPort = eff.getReceiver() + "." + eff.getReceiverPort();
      if (system.isEffected(senderPort, receiverPort)) {
        connectorChainFound = true;
      }
    }

    if (!connectorChainFound) {
      InconsistencyMissingEffector missingEffector = new InconsistencyMissingEffector(eff.getSender(), eff.getReceiver(), eff.getSenderPort(), eff.getReceiverPort());
      missingEffectors.add(missingEffector);
    }

  }

}
