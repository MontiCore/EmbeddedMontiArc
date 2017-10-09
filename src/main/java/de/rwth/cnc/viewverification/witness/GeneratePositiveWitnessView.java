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
package de.rwth.cnc.viewverification.witness;

import java.util.*;

import de.rwth.cnc.model.*;

public class GeneratePositiveWitnessView {

  public static CnCView getViewForPositiveWitness(CnCArchitecture arch, CnCView view) {

    CnCView witness = new CnCView();
    witness.setName("WitnessForSatisfaction");
    witness.setComment("// " + "The C&C model " + arch.getName() + " satisfies the view " + view.getName() + ".");

    if (view.getComponents().size() > 0) {
      Set<Connection> cons = new LinkedHashSet<Connection>();

      for (Connection c : view.getConnections()) {
        cons.addAll(computeConnectorChain(c, arch));
      }
      for (Connection c : view.getEffectors()) {
        cons.addAll(computeEffectorChain(c, arch));
      }

      Set<String> cmpNames = new LinkedHashSet<String>();
      cmpNames.addAll(view.getComponentNames());
      for (Connection c : cons) {
        cmpNames.add(c.getSender());
        cmpNames.add(c.getReceiver());
      }

      Component leastCommonParent = arch.getLeastCommonParent(cmpNames);

      Component parentCmp = new Component();
      parentCmp.setName(leastCommonParent.getName());
      witness.addComponent(parentCmp);
      List<String> tlc = new ArrayList<String>();
      tlc.add(parentCmp.getName());
      witness.setTopLevelComponentNames(tlc);

      for (String cmpName : cmpNames) {
        witness.addComponentWithIntermediateLayers(arch, cmpName);
      }

      addComponentsAndConnectors(witness, arch, new ArrayList<Connection>(cons));

      for (String cmpName : view.getComponentNames()) {
        Component cmpView = view.getComponent(cmpName);
        Component cmpArch = arch.getComponent(cmpName);
        Component cmpWitness = witness.getComponent(cmpName);
        addMatchingPortsOfCmpFromArchitectureToWitness(cmpWitness, cmpArch, cmpView);
      }
    }

    return witness;
  }

  private static void addMatchingPortsOfCmpFromArchitectureToWitness(Component cmpWitness, Component cmpArch, Component cmpView) {
  /*  // OLD SOURCE:
    for (Port pView : cmpView.getPorts()) {
      if (pView.isUnnamed()) {
        // check if witness already contains corresponding port
        boolean portExistsInWitness = false;
        for (Port pWitness : cmpWitness.getPorts()) {
          if (pView.getDirection().equals(pWitness.getDirection()) && pView.getType().equals(pWitness.getType())) {
            portExistsInWitness = true;
            break;
          }
        }
        if (!portExistsInWitness) {
          // find corresponding port in architecture and add it to the witness
          Port corrPortArch = null;
          for (Port pArch : cmpArch.getPorts()) {
            if (pView.getDirection().equals(pArch.getDirection()) && pView.getType().equals(pArch.getType())) {
              corrPortArch = pArch;
              break;
            }
          }
          cmpWitness.addPort(corrPortArch.clone());
        }
      }
      else {
        // if port is named we can just look it up in the architecture
        // and copy it to the view (if it doesn't contain it already)
        Port pArch = cmpArch.getPort(pView.getName());
        if (cmpWitness.getPort(pView.getName()) == null) {
          cmpWitness.addPort(pArch.clone());
        }
      }
    }
  /*  */

    for (Port port : cmpView.getPorts()) {
      Port p = port.getMappedPort();
      cmpWitness.addPort(p.clone());
    }
  }

  /**
   * adds concrete connectors to the view creates target and source components
   * and ports if they do not exist yet
   *
   * @param witness
   * @param arch
   * @param connectors list of concrete connectors
   */
  private static void addComponentsAndConnectors(CnCView witness, CnCArchitecture arch, List<Connection> connectors) {
    if (connectors.size() > 0) {
      Connection c = connectors.remove(0);

      witness.addComponentWithIntermediateLayers(arch, c.getSender());
      WitnessGeneratorHelper.copyPortToView(arch, witness, c.getSender(), c.getSenderPort());

      witness.addComponentWithIntermediateLayers(arch, c.getReceiver());
      WitnessGeneratorHelper.copyPortToView(arch, witness, c.getReceiver(), c.getReceiverPort());

      if (c instanceof Effector) {
        witness.addEffector((Effector) c.clone());
      }
      else {
        witness.addConnection(c.clone());
      }

      addComponentsAndConnectors(witness, arch, connectors);
    }
  }

  /**
   * uses BFS to compute a shortest chain of connectors and effectors if one
   * exists for the abstract connecor
   *
   * @param c
   * @param arch
   * @return
   */
  private static List<Connection> computeEffectorChain(Connection c, CnCArchitecture arch) {

    Queue<List<Connection>> workList = new LinkedList<List<Connection>>();
    Set<Connection> seen = new LinkedHashSet<Connection>();

    // add all connectors from sender to 1-element chains worklist
    if (c.isComponentToComponent() || c.isComponentToPort()) {
      for (Port p : arch.getComponent(c.getSender()).getPorts()) {
        for (Connection nc : arch.getOutGoingConnectorsEffectors(c.getSender() + "." + p.getName())) {
          List<Connection> chain = new ArrayList<Connection>();
          chain.add(nc);
          seen.add(nc);
          workList.add(chain);
        }
      }
    }
    else {
      for (Connection nc : arch.getOutGoingConnectorsEffectors(c.getSender() + "." + c.getSenderPort())) {
        if (!seen.contains(nc)) {
          List<Connection> chain = new ArrayList<Connection>();
          chain.add(nc);
          workList.add(chain);
        }
      }
    }

    while (!workList.isEmpty()) {
      List<Connection> chain = workList.poll();
      // check last connection in chain
      Connection cc = chain.get(chain.size() - 1);
      // check whether we are done
      if (cc.getReceiver().equals(c.getReceiver()) && (c.isComponentToComponent() || c.isPortToComponent() || cc.getReceiverPort().equals(c.getReceiverPort()))) {
        return chain;
      }
      for (Connection nc : arch.getOutGoingConnectorsEffectors(cc.getReceiver() + "." + cc.getReceiverPort())) {
        if (!seen.contains(nc)) {
          List<Connection> extChain = new ArrayList<Connection>(chain);
          extChain.add(nc);
          workList.add(extChain);
        }
      }
    }

    return new ArrayList<Connection>();
  }

  /**
   * uses BFS to compute a shortest chain of connectors and effectors if one
   * exists for the abstract connecor
   *
   * @param c
   * @param arch
   * @return
   */
  private static List<Connection> computeConnectorChain(Connection c, CnCArchitecture arch) {

    Queue<List<Connection>> workList = new LinkedList<List<Connection>>();
    Set<Connection> seen = new LinkedHashSet<Connection>();

    // add all connectors from sender to 1-element chains worklist
    if (c.isComponentToComponent() || c.isComponentToPort()) {
      for (Port p : arch.getComponent(c.getSender()).getPorts()) {
        for (Connection nc : arch.getOutGoingConnectors(c.getSender() + "." + p.getName())) {
          List<Connection> chain = new ArrayList<Connection>();
          chain.add(nc);
          seen.add(nc);
          workList.add(chain);
        }
      }
    }
    else {
      for (Connection nc : arch.getOutGoingConnectors(c.getSender() + "." + c.getSenderPort())) {
        if (!seen.contains(nc)) {
          List<Connection> chain = new ArrayList<Connection>();
          chain.add(nc);
          workList.add(chain);
        }
      }
    }

    while (!workList.isEmpty()) {
      List<Connection> chain = workList.poll();
      // check last connection in chain
      Connection cc = chain.get(chain.size() - 1);
      // check whether we are done
      if (cc.getReceiver().equals(c.getReceiver()) && (c.isComponentToComponent() || c.isPortToComponent() || cc.getReceiverPort().equals(c.getReceiverPort()))) {
        return chain;
      }
      for (Connection nc : arch.getOutGoingConnectors(cc.getReceiver() + "." + cc.getReceiverPort())) {
        if (!seen.contains(nc)) {
          List<Connection> extChain = new ArrayList<Connection>(chain);
          extChain.add(nc);
          workList.add(extChain);
        }
      }
    }

    return new ArrayList<Connection>();
  }

}
