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
      if (system.getComponentNames().contains(eff.getSender()) && system.getComponentNames().contains(eff.getReceiver())) {
        checkEffector(eff);
      }
    }

    return missingEffectors.isEmpty();
  }

  private void checkEffector(Effector eff) {
    boolean connectorChainFound = false;

    if (eff.isComponentToComponent()) {
      for (Port p : system.getComponent(eff.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = eff.getSender() + "." + p.getName();
        if (system.isEffected(senderPort, eff.getReceiver())) {
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
      for (Port p : system.getComponent(eff.getSender()).getPorts()) {
        // check for all outgoing ports
        String senderPort = eff.getSender() + "." + p.getName();
        String receiverPort = eff.getReceiver() + "." + eff.getReceiverPort();
        if (system.isEffected(senderPort, receiverPort)) {
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
