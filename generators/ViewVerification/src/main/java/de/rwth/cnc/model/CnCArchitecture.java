/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.model;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import de.rwth.cnc.viewverification.VerificationHelper;

public class CnCArchitecture extends CnCView implements Cloneable {

  private Map<String, Set<String>> directlyConnectedPorts;
  private Map<String, Set<String>> directlyEffectedPorts;
  private Map<String, Set<Connection>> connEffs;
  private Map<String, Set<Connection>> conns;

  /**
   * retrieves all targets of the <code>senderPort</code> that is equal to
   * <code>c.getSender() + "." + c.getSenderPort()</code>
   *
   * @param senderCmpAndPort String of the form "MyComponent.myPort"
   * @return
   */
  public Set<String> getReceivingCmpsAndPorts(String senderCmpAndPort) {
    if (directlyConnectedPorts == null) {
      directlyConnectedPorts = new HashMap<String, Set<String>>();
      for (Connection c : getConnections()) {
        String sender = c.getSender() + "." + c.getSenderPort();
        Set<String> targets = directlyConnectedPorts.get(sender);
        if (targets == null) {
          targets = new LinkedHashSet<String>();
          directlyConnectedPorts.put(sender, targets);
        }
        targets.add(c.getReceiver() + "." + c.getReceiverPort());
      }
    }
    if (directlyConnectedPorts.get(senderCmpAndPort) == null) {
      return new LinkedHashSet<String>();
    }
    return directlyConnectedPorts.get(senderCmpAndPort);
  }

  /**
   * copy of the above with modifications to also consider effectors
   *
   * @param senderCmpAndPort
   */
  public Set<String> getEffectedCmpsAndPorts(String senderCmpAndPort) {
    directlyEffectedPorts = null; //recalculate since we calculate OnlyEffectors below very similarly!
    if (directlyEffectedPorts == null) {
      directlyEffectedPorts = new HashMap<String, Set<String>>();
      List<Connection> connEffs = new ArrayList<>();
      connEffs.addAll(getConnections());
      connEffs.addAll(getEffectors());
      for (Connection e : connEffs) {
        String sender = e.getSender() + "." + e.getSenderPort();
        Set<String> targets = directlyEffectedPorts.get(sender);
        if (targets == null) {
          targets = new LinkedHashSet<String>();
          directlyEffectedPorts.put(sender, targets);
        }
        targets.add(e.getReceiver() + "." + e.getReceiverPort());
      }
    }
    if (directlyEffectedPorts.get(senderCmpAndPort) == null) {
      return new LinkedHashSet<String>();
    }
    return directlyEffectedPorts.get(senderCmpAndPort);
  }

  /**
   * copy of the above with modifications to only consider effectors
   *
   * @param senderCmpAndPort
   */
  public Set<String> getEffectedCmpsAndPortsOnlyEffectors(String senderCmpAndPort) {
    directlyEffectedPorts = null; //recalculate!
    if (directlyEffectedPorts == null) {
      directlyEffectedPorts = new HashMap<String, Set<String>>();
      List<Effector> effs = new ArrayList<Effector>();
      effs.addAll(getEffectors());
      for (Effector e : effs) {
        String sender = e.getSender() + "." + e.getSenderPort();
        Set<String> targets = directlyEffectedPorts.get(sender);
        if (targets == null) {
          targets = new LinkedHashSet<String>();
          directlyEffectedPorts.put(sender, targets);
        }
        targets.add(e.getReceiver() + "." + e.getReceiverPort());
      }
    }
    if (directlyEffectedPorts.get(senderCmpAndPort) == null) {
      return new LinkedHashSet<String>();
    }
    return directlyEffectedPorts.get(senderCmpAndPort);
  }

  public boolean isEffected(String senderPort, String receiver) {
    return isEffectedTransitive(senderPort, receiver);
  }

  /**
   * checks for transitively effected ports using BFS without recursion and with
   * cycle detection
   *
   * @param senderAndPort
   * @param receiver
   * @return
   */
  private boolean isEffectedTransitive(String senderAndPort, String receiver) {
    Set<String> seen = new LinkedHashSet<String>();
    Queue<String> effected = new LinkedList<String>();
    if (checkMatch(senderAndPort, receiver)) {
      return true;
    }
    else {
      seen.add(senderAndPort);
      // add effected ports of senderAndPort if not seen so far
      Set<String> nextEffected = getEffectedCmpsAndPorts(senderAndPort);
      nextEffected.removeAll(seen);
      seen.addAll(nextEffected);
      effected.addAll(nextEffected);
    }

    while (!effected.isEmpty()) {
      String candidate = effected.poll();
      if (checkMatch(candidate, receiver)) {
        return true;
      }
      else {
        // add effected ports of candidate if not seen so far
        Set<String> nextEffected = getEffectedCmpsAndPorts(candidate);
        nextEffected.removeAll(seen);
        seen.addAll(nextEffected);
        effected.addAll(nextEffected);
      }
    }
    return false;
  }

  private boolean checkMatch(String senderAndPort, String receiver) {
    if (receiver.contains(".")) { // case receiver is cmp.port
      if (senderAndPort.equals(receiver)) {
        return true;
      }
    }
    else { // case receiver is cmp only
      if (senderAndPort.startsWith(receiver + ".")) {
        return true;
      }
    }
    return false;
  }

  /**
   * works for receivers that are either components or components with a port
   * (distinguished by a dot between component and port)
   *
   * @param senderPort
   * @param receiver
   * @return
   */
  public boolean isConnected(String senderPort, String receiver) {
    return isConnected(senderPort, senderPort, receiver);
  }

  private boolean isConnected(String initialSenderPort, String senderPort, String receiver) {
    if (checkMatch(senderPort, receiver)) {
      return true;
    }

    for (String tgtPort : getReceivingCmpsAndPorts(senderPort)) {
      if (!tgtPort.equals(initialSenderPort)) {
        if (isConnected(initialSenderPort, tgtPort, receiver)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * returns the path containing parent and child component
   * (every component except the child contains its child on the path)
   *
   * @param parentName
   * @param childName
   * @return
   */
  public List<Component> getArchPath(String parentName, String childName) {
    if (this.getComponent(parentName) == null)
      parentName = VerificationHelper.uncapitalize(parentName);

    assert this.getComponent(parentName) != null;

    Component parent = new Component();
    parent.setName(parentName);

    if (parentName.equals(childName) || parentName.equals(VerificationHelper.uncapitalize(childName))) {
      ArrayList<Component> path = new ArrayList<Component>();
      path.add(parent);
      return path;
    }
    for (String nextParentName : this.getComponent(parentName).getContainedComponents()) {
      List<Component> path = getArchPath(nextParentName, childName);
      if (path != null) {
        parent.addContainedComponent(nextParentName);
        path.add(0, parent);
        return path;
      }
    }
    return null;
  }

  /**
   * compute the set of connectors and effectors from a source component and
   * port
   *
   * @param srcCmpAndPort
   * @return
   */
  public Set<Connection> getOutGoingConnectorsEffectors(String srcCmpAndPort) {
    if (connEffs == null) {
      connEffs = new HashMap<String, Set<Connection>>();
      List<Connection> cs = new ArrayList<Connection>(getConnections());
      cs.addAll(getEffectors());
      for (Connection c : cs) {
        String src = c.getSender() + "." + c.getSenderPort();
        Set<Connection> ces = connEffs.get(src);
        if (ces == null) {
          ces = new LinkedHashSet<Connection>();
          connEffs.put(src, ces);
        }
        ces.add(c);
      }
    }

    if (connEffs.get(srcCmpAndPort) == null) {
      return new LinkedHashSet<Connection>();
    }
    return connEffs.get(srcCmpAndPort);
  }

  /**
   * compute the set of connectors from a source component and
   * port
   *
   * @param srcCmpAndPort
   * @return
   */
  public Set<Connection> getOutGoingConnectors(String srcCmpAndPort) {
    if (conns == null) {
      conns = new HashMap<String, Set<Connection>>();
      for (Connection c : getConnections()) {
        String src = c.getSender() + "." + c.getSenderPort();
        Set<Connection> ces = conns.get(src);
        if (ces == null) {
          ces = new LinkedHashSet<Connection>();
          conns.put(src, ces);
        }
        ces.add(c);
      }
    }

    if (conns.get(srcCmpAndPort) == null) {
      return new LinkedHashSet<Connection>();
    }
    return conns.get(srcCmpAndPort);
  }

  @Override
  public CnCArchitecture clone() {
    CnCArchitecture clone = new CnCArchitecture();
    for (Component c : components) {
      clone.addComponent(c.clone());
    }
    for (Connection c : connections) {
      clone.addConnection(c.clone());
    }
    for (Effector e : effectors) {
      clone.addEffector(e.clone());
    }
    clone.name = name;
    clone.packageName = packageName;
    clone.comment = comment;
    clone.setTopLevelComponentNames(topLevelComponentNames);
    return clone;
  }
}
