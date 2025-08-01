/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.model;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import de.rwth.cnc.viewverification.VerificationHelper;

/**
 * this class represents a CnC View
 */
public class CnCView implements Cloneable {

  protected String name = null;
  protected String comment;
  protected Path fileOrigin;
  protected String packageName;

  protected List<Component> components = new LinkedList<>();
  protected List<String> topLevelComponentNames = new LinkedList<>();
  protected List<Connection> connections = new LinkedList<>();
  protected List<Effector> effectors = new LinkedList<>();

  public String getPackageName() {
    return packageName;
  }

  public void setPackageName(String packageName) {
    this.packageName = packageName;
  }

  public void setFileOrigin(Path filePath) {
    fileOrigin = filePath;
  }

  public Path getFileOrigin() {
    return fileOrigin;
  }

  public List<Component> getComponents() {
    return components;
  }

  public void addComponent(Component cmp) {
    components.add(cmp);
  }

  /**
   * calculate independent components, i.e., components that are on the same
   * level of hierarchy and have the same view-parent
   *
   * @return list of list where each simple list represents an independent set
   */
  public List<List<String>> getIndependentComponentSets() {
    List<List<String>> independentComponentSets = new ArrayList<List<String>>();
    for (Component cmp : this.components) {
      List<String> cmpList = cmp.getContainedComponents();
      if (cmpList.size() > 1) {
        independentComponentSets.add(cmpList);
      }
    }
    if (topLevelComponentNames.size() > 1) {
      independentComponentSets.add(topLevelComponentNames);
    }
    return independentComponentSets;
  }

  /**
   * calculate independent components pairs
   *
   * @return list of list where each simple list represents an independent pair
   */
  public List<List<String>> getIndependentComponentPairs() {
    List<List<String>> independentComponentPairs = new ArrayList<List<String>>();
    for (List<String> indSetOrig : getIndependentComponentSets()) {
      // first copy indSetOrig to indSet because we will remove elements
      // (this would change containment!!! because of reuse of lists)
      List<String> indSet = new ArrayList<String>();
      indSet.addAll(indSetOrig);
      while (indSet.size() > 2) {
        String head = indSet.remove(0);
        for (String other : indSet) {
          List<String> indPair = new ArrayList<String>();
          indPair.add(head);
          indPair.add(other);
          independentComponentPairs.add(indPair);
        }
      }
      independentComponentPairs.add(indSet);
    }
    return independentComponentPairs;
  }

  public List<Connection> getConnections() {
    return connections;
  }

  public void addConnection(Connection conn) {
    assert !(conn instanceof Effector) : "Add Effectors with addEffector()!";
    this.connections.add(conn);
  }

  public List<Effector> getEffectors() {
    return effectors;
  }

  public void addEffector(Effector eff) {
    this.effectors.add(eff);
  }

  public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  /**
   * compute set of all port names in the view
   *
   * @return
   */
  public Set<String> getPortNames() {
    Set<String> portNames = new LinkedHashSet<String>();

    // collect port names from components
    for (Component cmp : this.components) {
      for (Port port : cmp.getPorts()) {
        if (!port.isUnnamed()) {
          portNames.add(port.getFullName());
        }
        else
          portNames.add("?");
      }
    }
    // also collect port names from connections
      /*
      for (Connection conn : this.connections) {
        if (conn.isComponentToPort()) {
          portNames.add(conn.getReceiverPort());
        }
        else if (conn.isPortToComponent()) {
          portNames.add(conn.getSenderPort());
        }
        else if (conn.isPortToPort()) {
          portNames.add(conn.getSenderPort());
          portNames.add(conn.getReceiverPort());
        }
      }
      */

    return portNames;
  }

  public Set<Port> getPorts() {
    Set<Port> ports = new LinkedHashSet<Port>();

    // collect port names from components
    for (Component cmp : this.components) {
      for (Port port : cmp.getPorts())
        ports.add(port);
    }

    return ports;
  }

  /**
   * collect all types used on ports of the components in this view
   *
   * @return
   */
  public Set<String> getPortTypes() {
    Set<String> types = new LinkedHashSet<String>();
    for (Component cmp : this.components) {
      types.addAll(cmp.getPortTypes());
    }
    return types;
  }

  /**
   * returns all names of components (without those mentioned in connectors)
   *
   * @return
   */
  public Set<String> getComponentNames() {
    return getComponentNames(false);
  }

  /**
   * returns all names of components (optionally including names only mentioned
   * in connectors)
   *
   * @param includeConsEffs
   * @return
   */
  public Set<String> getComponentNames(boolean includeConsEffs) {
    Set<String> componentNames = new LinkedHashSet<String>();
    // collect names from components
    for (Component component : components) {
      componentNames.add(component.getName());
    }
    // also collect names from connectors
    if (includeConsEffs) {
      for (Connection conn : this.connections) {
        componentNames.add(conn.getSender());
        componentNames.add(conn.getReceiver());
      }
      for (Connection eff : this.effectors) {
        componentNames.add(eff.getSender());
        componentNames.add(eff.getReceiver());
      }
    }
    return componentNames;
  }

  /**
   * copies all the component names to a local list
   *
   * @param cmpNames
   */
  public void setTopLevelComponentNames(List<String> cmpNames) {
    assert cmpNames.stream().allMatch(x -> Character.isUpperCase(x.charAt(0))) : "TopLevelComponentNames have to be capitalized!";
    this.topLevelComponentNames = new ArrayList<String>();
    this.topLevelComponentNames.addAll(cmpNames);
  }

  /**
   * find the component in the view for the given name
   *
   * @param cmpName
   * @return component or null if not found in view
   */
  public Component getComponent(String cmpName) {
    for (Component cmp : components) {
      if (cmp.getName().equals(cmpName)) {
        return cmp;
      }
    }
    return null;
  }

  /**
   * returns the component that owns exactly this port object null otherwise
   *
   * @param p
   * @return
   */
  public Component getComponent(Port p) {
    for (Component c : components) {
      if (c.getPorts().contains(p)) {
        return c;
      }
    }
    return null;
  }

  public void setComment(String comment) {
    this.comment = comment;
  }

  public String getComment() {
    return comment;
  }

  /**
   * check that parentCmp is a parent of childCmp in the architecture also true
   * if parentCmp is childCmp
   *
   * @param parentCmp
   * @param childCmp
   * @return
   */
  public boolean isParentChild(String parentCmp, String childCmp) {
    // check that components exist in this view
    Set<String> cmpNames = getComponentNames();
    if (!cmpNames.contains(parentCmp) || !cmpNames.contains(childCmp)) {
      return false;
    }

    if (parentCmp.equals(childCmp)) {
      return true;
    }
    for (String child : getComponent(parentCmp).getContainedComponents()) {
      if (child.equals(childCmp)) {
        return true;
      }
      else {
        if (isParentChild(child, childCmp)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * find the least common parent component (can be one of the given components
   * if it contains all others) uses pairwise method
   *
   * @param cmps
   * @return null if no parent can be found
   */
  public Component getLeastCommonParent(Set<String> cmps) {
    if (cmps.size() == 0) {
      return null;
    }
    else if (cmps.size() == 1) {
      Component c = getComponent(cmps.iterator().next());
      assert c != null : "this shouldnt happen";
      return c;
    }
    else {
      Set<String> myCmps = new LinkedHashSet<String>();
      myCmps.addAll(cmps);
      Iterator<String> it = cmps.iterator();
      String cmp1 = it.next();
      String cmp2 = it.next();
      // replace pairs of components by their least common parent
      myCmps.remove(cmp1);
      myCmps.remove(cmp2);

      myCmps.add(getLeastCommonParent(cmp1, cmp2).getName());
      // call recursively with #components - 1
      return getLeastCommonParent(myCmps);
    }
  }

  /**
   * find the least common parent component (can be one of the given components
   * if it contains the other one) starts from top components and descends tree
   *
   * @param cmp1
   * @param cmp2
   * @return
   */
  public Component getLeastCommonParent(String cmp1, String cmp2) {
    if (cmp1.equals(cmp2)) {
      return getComponent(cmp2);
    }
    for (String topCmp : topLevelComponentNames) {
      Component lcp = getLeastCommonParent(topCmp, cmp1, cmp2);
      if (lcp != null) {
        return lcp;
      }
    }
    return null;
  }

  public Component getDirectParent(String childName) {
    for (Component c : components) {
      if (c.getContainedComponents().contains(childName)) {
        return c;
      }
    }
    return null;
  }

  private Component getLeastCommonParent(String topCmp, String cmp1, String cmp2) {
    Component parent = getComponent(topCmp);
    for (String child : parent.getContainedComponents()) {
      Component lcp = getLeastCommonParent(child, cmp1, cmp2);
      if (lcp != null) {
        return lcp;
      }
    }
    if (isParentChild(topCmp, cmp1) && isParentChild(topCmp, cmp2)) {
      return parent;
    }
    return null;
  }

  public List<String> getTopLevelComponentNames() {
    return topLevelComponentNames;
  }

  public void addTopLevelComponentName(String name2) {
    assert Character.isUpperCase(name2.charAt(0)) : "TopLevelComponentNames have to be capitalized!";
    topLevelComponentNames.add(name2);
  }

  public void addTopLevelComponentName(String name2, boolean capitalize) {
    if (!capitalize)
      addTopLevelComponentName(name2);
    else
      topLevelComponentNames.add(VerificationHelper.capitalize(name2));
  }

  /**
   * adds the component with intermediate layers adds intermediate components
   * from the architecture to the view !! expects that there is just a single
   * topLevelComponent in the view !!
   *
   * @param arc
   * @param cmpName
   */
  public boolean addComponentWithIntermediateLayers(CnCArchitecture arc, String cmpName) {
    if (arc.getComponent(cmpName) == null)
      return false;

    if (!this.containsComponent(cmpName)) {
      List<Component> cmpPath = arc.getArchPath(this.getTopLevelComponentNames().get(0), cmpName);
      // find first component that is not contained already in system
      assert cmpPath != null;
      String lastParentCmpName = cmpPath.get(0).getName();
      while (this.containsComponent(cmpPath.get(0).getName())) {
        lastParentCmpName = cmpPath.get(0).getName();
        cmpPath.remove(0);
      }
      // add components
      this.getComponent(lastParentCmpName).addContainedComponent(cmpPath.get(0).getName());
      this.getComponents().addAll(cmpPath);
    }
    return true;
  }

  private boolean containsComponent(String cmpName) {
    return this.getComponent(cmpName) != null;
  }

  @Override
  public CnCView clone() {
    CnCView clone = new CnCView();
    clone.comment = comment;
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

  /**
   * removed the component from the view also removes its name from the list of
   * top components also removes its entry as sub components also removed all
   * dangling connections
   *
   * @param ctr
   */
  public void remove(Component ctr) {
    components.remove(ctr);

    if (topLevelComponentNames.contains(ctr.getName())) {
      topLevelComponentNames.remove(ctr.getName());
      topLevelComponentNames.addAll(ctr.getContainedComponents());
      assert topLevelComponentNames.stream().allMatch(x -> Character.isUpperCase(x.charAt(0))) : "TopLevelComponentNames have to be capitalized!";
    }

    for (Component parent : components) {
      if (parent.getContainedComponents().contains(ctr.getName())) {
        parent.getContainedComponents().remove(ctr.getName());
        parent.getContainedComponents().addAll(ctr.getContainedComponents());
      }
    }

    List<Connection> danglingConns = new ArrayList<Connection>();
    for (Connection c : connections) {
      if (c.getSender().equals(ctr.getName()) || c.getReceiver().equals(ctr.getName())) {
        danglingConns.add(c);
      }
    }
    connections.removeAll(danglingConns);
  }

  public void remove(Connection con) {
    connections.remove(con);
  }

  public void remove(Effector con) {
    effectors.remove(con);
  }

  /**
   * removes the port and all dangling connections
   *
   * @param p
   */
  public void remove(Port p) {
    Component cmp = getComponent(p);
    cmp.getPorts().remove(p);

    List<Connection> danglingConns = new ArrayList<Connection>();
    for (Connection conn : connections) {
      if ((conn.getSender().equals(cmp.getName()) && conn.getSenderPort().equals(p.getName())) || (conn.getReceiver().equals(cmp.getName()) && conn.getReceiverPort().equals(p.getName()))) {
        danglingConns.add(conn);
      }
    }
    connections.removeAll(danglingConns);
  }

  public void renameCmp(String cmpName, String newCmpName) {
    if (topLevelComponentNames.contains(cmpName)) {
      topLevelComponentNames.remove(cmpName);
      assert Character.isUpperCase(newCmpName.charAt(0)) : "TopLevelComponentNames have to be capitalized!";
      topLevelComponentNames.add(newCmpName);

    }

    for (Component cmp : components) {
      if (cmp.getName().equals(cmpName)) {
        cmp.setName(newCmpName);
      }
      if (cmp.getContainedComponents().contains(cmpName)) {
        cmp.getContainedComponents().remove(cmpName);
        cmp.getContainedComponents().add(newCmpName);
      }
    }
    for (Connection conn : connections) {
      if (conn.getSender().equals(cmpName)) {
        conn.setSender(newCmpName);
      }
      if (conn.getReceiver().equals(cmpName)) {
        conn.setReceiver(newCmpName);
      }
    }
    for (Effector conn : effectors) {
      if (conn.getSender().equals(cmpName)) {
        conn.setSender(newCmpName);
      }
      if (conn.getReceiver().equals(cmpName)) {
        conn.setReceiver(newCmpName);
      }
    }
  }

  public void renamePort(String cmpName, String portName, String newPortName) {
    getComponent(cmpName).getPort(portName).setName(newPortName);
    for (Connection conn : connections) {
      if (conn.getSender().equals(cmpName) && conn.getSenderPort() != null && conn.getSenderPort().equals(portName)) {
        conn.setSenderPort(newPortName);
      }
      if (conn.getReceiver().equals(cmpName) && conn.getReceiverPort() != null && conn.getReceiverPort().equals(portName)) {
        conn.setReceiverPort(newPortName);
      }
    }
  }

  /**
   * retrieves all components reachable by chains of connectors
   *
   * @param cmpName
   * @return
   */
  public Set<String> getReachableComponents(String cmpName, String pName, boolean alsoEffectors) {
    Set<String> reachableCmpsPorts = new LinkedHashSet<String>();
    Port p = getComponent(cmpName).getPort(pName);
    reachableCmpsPorts.addAll(getReachableComponents(new LinkedHashSet<String>(), cmpName, p.getName(), alsoEffectors));

    Set<String> reachableCmps = new LinkedHashSet<String>();
    for (String cmpsPort : reachableCmpsPorts) {
      reachableCmps.add(cmpsPort.split("\\.")[0]);
    }
    return reachableCmps;
  }

  public Set<String> getReachableComponents(String cmpName, boolean alsoEffectors) {
    Set<String> reachableCmpsPorts = new LinkedHashSet<String>();
    for (Port p : getComponent(cmpName).getPorts()) {
      reachableCmpsPorts.addAll(getReachableComponents(new LinkedHashSet<String>(), cmpName, p.getName(), alsoEffectors));
    }
    Set<String> reachableCmps = new LinkedHashSet<String>();
    for (String cmpsPort : reachableCmpsPorts) {
      reachableCmps.add(cmpsPort.split("\\.")[0]);
    }
    return reachableCmps;
  }

  /**
   * helper function avoiding circular inclusions
   *
   * @param cmpsPortsVisited
   * @param cmpName
   * @return
   */
  private Set<String> getReachableComponents(LinkedHashSet<String> cmpsPortsVisited, String cmpName, String pName, boolean alsoEffectors) {
    String cmpPort = cmpName + "." + pName;
    cmpsPortsVisited.add(cmpPort);
    for (Connection conn : connections) {
      if (conn.getSender().equals(cmpName) && conn.getSenderPort().equals(pName) && !cmpsPortsVisited.contains(conn.getReceiver() + "." + conn.getReceiverPort())) {
        cmpsPortsVisited.addAll(getReachableComponents(cmpsPortsVisited, conn.getReceiver(), conn.getReceiverPort(), alsoEffectors));
      }
    }

    if (alsoEffectors) {
      for (Effector eff : effectors) {
        if (eff.getSender().equals(cmpName) && eff.getSenderPort().equals(pName) && !cmpsPortsVisited.contains(eff.getReceiver() + "." + eff.getReceiverPort())) {
          cmpsPortsVisited.addAll(getReachableComponents(cmpsPortsVisited, eff.getReceiver(), eff.getReceiverPort(), alsoEffectors));
        }
      }

    }
    return cmpsPortsVisited;
  }

}
