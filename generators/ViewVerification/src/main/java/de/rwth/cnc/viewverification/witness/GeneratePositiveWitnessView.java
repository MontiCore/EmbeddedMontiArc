/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.witness;

import java.util.*;

import de.rwth.cnc.model.*;
import de.rwth.cnc.viewverification.VerificationHelper;

public class GeneratePositiveWitnessView {

  public static CnCView getViewForPositiveWitness(final CnCArchitecture architecture, final CnCView view) {

    CnCArchitecture arch = architecture.clone();
    CnCView witness = new CnCView();
    witness.setName("Witness_" + arch.getName());
    witness.setComment("// " + "The C&C model " + arch.getName() + " satisfies the view " + view.getName() + ".");
    witness.setPackageName(view.getPackageName());

    if (view.getComponents().size() > 0) {
      addInner(arch, view, witness);
    }

    return witness;
  }

  private static void addInner(final CnCArchitecture arch, final CnCView view, CnCView witness) {
    Set<Connection> cons = new LinkedHashSet<Connection>();

    addConnectorsToCons(arch, view, cons);
    addEffectorsToCons(arch, view, cons);
    Set<String> cmpNames = getComponentsFromViewAndCons(view, cons);

    Component leastCommonParent = arch.getLeastCommonParent(cmpNames);

    //capitalize the component name to handle the difference between view and model
    arch.renameCmp(leastCommonParent.getName(), VerificationHelper.capitalize(leastCommonParent.getName()));
    leastCommonParent.setName(VerificationHelper.capitalize(leastCommonParent.getName()));

    setTopLevelOfWitness(witness, leastCommonParent);

    for (String cmpName : cmpNames) {
      if (!witness.addComponentWithIntermediateLayers(arch, cmpName)) {
        witness.addComponentWithIntermediateLayers(arch, VerificationHelper.uncapitalize(cmpName));
      }
    }

    addComponentsAndConnectors(witness, arch, new ArrayList<>(cons));

    for (String cmpName : view.getComponentNames()) {
      Component cmpView = view.getComponent(cmpName);
      Component cmpArch = arch.getComponent(cmpName);
      if (cmpArch == null) {
        cmpArch = arch.getComponent(VerificationHelper.capitalize(cmpName));
      }
      Component cmpWitness = witness.getComponent(cmpName);
      if (cmpWitness == null) {
        cmpWitness = witness.getComponent(VerificationHelper.capitalize(cmpName));
      }
      addMatchingPortsOfCmpFromArchitectureToWitness(cmpWitness, cmpArch, cmpView);
    }
  }

  private static void addConnectorsToCons(final CnCArchitecture arch, final CnCView view, Set<Connection> cons) {
    for (Connection c : view.getConnections()) {
      cons.addAll(computeConnectorChain(c, arch));
    }
  }

  private static void addEffectorsToCons(final CnCArchitecture arch, final CnCView view, Set<Connection> cons) {
    for (Connection con : view.getEffectors()) {
      Connection c = con.clone();
      List<Connection> connections = computeEffectorChain(c, arch);

      if (connections.size() == 0) {
        // iterate all capitalized and uncapitalized possibilities,
        // but only if nothing was found
        boolean bothLowDone = false;
        if (Character.isUpperCase(c.getSender().charAt(0))) {
          Connection connection = c.clone();
          connection.setSender(VerificationHelper.uncapitalize(connection.getSender()));
          connections.addAll(computeEffectorChain(connection, arch));
          if (connections.size() == 0)
            if (Character.isUpperCase(c.getReceiver().charAt(0))) {
              connection.setReceiver(VerificationHelper.uncapitalize(connection.getReceiver()));
              connections.addAll(computeEffectorChain(connection, arch));
              bothLowDone = true;
            }
        }
        if (connections.size() == 0)
          if (Character.isUpperCase(c.getReceiver().charAt(0))) {
            Connection connection = c.clone();
            connection.setReceiver(VerificationHelper.uncapitalize(connection.getReceiver()));
            connections.addAll(computeEffectorChain(connection, arch));
            if (connections.size() == 0)
              if (!bothLowDone && Character.isUpperCase(c.getSender().charAt(0))) {
                connection.setSender(VerificationHelper.uncapitalize(connection.getSender()));
                connections.addAll(computeEffectorChain(connection, arch));
              }
          }
      }

      cons.addAll(connections);
    }
  }

  private static Set<String> getComponentsFromViewAndCons(final CnCView view, Set<Connection> cons) {
    Set<String> cmpNames = new LinkedHashSet<String>();
    cmpNames.addAll(view.getComponentNames());
    for (Connection c : cons) {
      cmpNames.add(c.getSender());
      cmpNames.add(c.getReceiver());
    }
    return cmpNames;
  }

  private static void setTopLevelOfWitness(CnCView witness, final Component leastCommonParent) {
    Component parentCmp = new Component();
    parentCmp.setName(leastCommonParent.getName());
    witness.addComponent(parentCmp);
    List<String> tlc = new ArrayList<String>();
    tlc.add(parentCmp.getName());
    witness.setTopLevelComponentNames(tlc);
  }

  private static void addMatchingPortsOfCmpFromArchitectureToWitness(Component cmpWitness, Component cmpArch, Component cmpView) {
    // OLD SOURCE:
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
            if (pView.getDirection().equals(pArch.getDirection()))

              if (pView.isTyped()) {
                if (pView.getType().equals(pArch.getType())) {
                  corrPortArch = pArch;
                  break;
                }
              }
              else {
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
        if (pArch == null)
          pArch = pView.getMappedPort();
        if (cmpWitness.getPort(pView.getName()) == null) {
          cmpWitness.addPort(pArch.clone());
        }
      }
    }

    // alternatively:
    // more efficient but doesnt consider if a port is already in the witness.
    //    assert cmpWitness != null;
    //    assert cmpView != null;
    //    for (Port port : cmpView.getPorts()) {
    //      Port p = port.getMappedPort();
    //      assert p != null;
    //      cmpWitness.addPort(p.clone());
    //    }
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

    if (arch.getComponent(c.getReceiver()) == null || arch.getComponent(c.getSender()) == null) {
      System.out.println("computeEffectorChain: Some component does not exist: " + c);
      return new ArrayList<Connection>();
    }

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
      for (Connection nc : arch.getOutGoingConnectorsEffectors(cc.getFullReceiver())) {
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
          seen.add(nc); //fk

          workList.add(chain);
        }
      }
    }

    while (!workList.isEmpty()) {
      List<Connection> chain = workList.poll();
      // check last connection in chain
      Connection cc = chain.get(chain.size() - 1);
      // check whether we are done
      if (cc.getReceiver().equals(c.getReceiver())) {
        if (c.isComponentToComponent() || (c.isComponentToPort() && cc.getReceiverPort().equals(c.getReceiverPort())))
          return chain;
        if (c.isPortToComponent())
          return chain;
      }
      for (Connection nc : arch.getOutGoingConnectors(cc.getReceiver() + "." + cc.getReceiverPort())) {
        if (!seen.contains(nc)) {
          List<Connection> extChain = new ArrayList<Connection>(chain);
          extChain.add(nc);
          seen.add(nc); //fk

          workList.add(extChain);
        }
      }
    }

    return new ArrayList<Connection>();
  }

}
