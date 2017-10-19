/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.cnc.model;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class Component implements Cloneable {

  private String name = "";
  private String componentType = "";

  private List<Port> ports = new ArrayList<Port>();

  private List<String> directlyContainedComponents = new ArrayList<String>();

  private boolean markedInterfaceComplete = false;
  // it is called "marked atomic" since simply "atomic" sounds like a 
  // property that could have been derived from directly contained components 
  private boolean markedAtomic = false;

  public String getComponentType() {
    return componentType;
  }

  public void setComponentType(String componentType) {
    this.componentType = componentType;
  }

  /**
   * directly contained components
   *
   * @return
   */
  public List<String> getContainedComponents() {
    return directlyContainedComponents;
  }

  public void addContainedComponent(String containedComponent) {
    if (!directlyContainedComponents.contains(containedComponent)) {
      this.directlyContainedComponents.add(containedComponent);
    }
  }

  public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public List<Port> getPorts() {
    return ports;
  }

  public void addPort(Port port) {
    //dont add a port twice
    if (!port.isUnnamed()) {
      if (!getPortNames().contains(port.getName()))
        this.ports.add(port);
    }
    else
      this.ports.add(port);
  }

  public boolean containsComponents() {
    return directlyContainedComponents.size() > 0;
  }

  public boolean hasPorts() {
    return ports.size() > 0;
  }

  public boolean isMarkedInterfaceComplete() {
    return markedInterfaceComplete;
  }

  public void setMarkedInterfaceComplete(boolean markedInterfaceComplete) {
    this.markedInterfaceComplete = markedInterfaceComplete;
  }

  public boolean isMarkedAtomic() {
    return markedAtomic;
  }

  public void setMarkedAtomic(boolean markedAtomic) {
    this.markedAtomic = markedAtomic;
  }

  public Port getPort(String pName) {
    for (Port p : ports) {
      if (p.getName() != null && p.getName().equals(pName)) {
        return p;
      }
    }
    return null;
  }

  @Override
  protected Component clone() {
    Component clone = new Component();
    clone.directlyContainedComponents.addAll(directlyContainedComponents);
    clone.markedInterfaceComplete = markedInterfaceComplete;
    clone.markedAtomic = markedAtomic;
    clone.componentType = componentType;
    clone.name = name;
    for (Port p : ports) {
      clone.addPort(p.clone());
    }
    return clone;
  }

  public Set<String> getPortTypes() {
    Set<String> types = new LinkedHashSet<String>();
    for (Port port : ports) {
      if (!port.isUntyped()) {
        types.add(port.getType());
      }
      else
        types.add("?");
    }
    return types;
  }

  public Set<String> getPortNames() {
    Set<String> names = new LinkedHashSet<String>();
    for (Port port : ports) {
      if (!port.isUnnamed()) {
        names.add(port.getName());
      }
      else
        names.add("?");
    }
    return names;
  }

  @Override
  public String toString() {
    return getName();
  }

}
