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
package de.rwth.cnc.model;

import java.util.Optional;

import de.monticore.symboltable.types.JTypeSymbol;
import de.monticore.symboltable.types.references.JTypeReference;

public class Port implements Cloneable {

  private String name;
  private String type;
  private Optional<JTypeReference<? extends JTypeSymbol>> typeReference;
  private Direction direction;
  private Component component;
  private Port mappedPort;

  public void setName(String name) {
    assert !name.contains(".") : "Port name must not be the full name!";
    this.name = name;
  }

  public void setType(String type) {
    this.type = type;
  }

  public void setTypeReference(Optional<JTypeReference<? extends JTypeSymbol>> typeReference) {
    this.typeReference = typeReference;
  }

  public Optional<JTypeReference<? extends JTypeSymbol>> getTypeReference() {
    return typeReference;
  }

  public void setMappedPort(Port mappedPort) {
    this.mappedPort = mappedPort;
  }

  public Port getMappedPort() {
    return mappedPort;
  }

  public void setDirection(Direction direction) {
    this.direction = direction;
  }

  public void setComponent(Component component) {
    this.component = component;
  }

  public String getName() {
    return name;
  }

  public String getFullName() {
    return getComponent().getName() + "." + getName();
  }

  public String getType() {
    return type;
  }

  public Direction getDirection() {
    return direction;
  }

  public Component getComponent() {
    return component;
  }

  public boolean isUnnamed() {
    return (name == null || name.length() == 0 || name == "?");
  }

  public boolean isNamed() {
    return !isUnnamed();
  }

  public boolean isUntyped() {
    return (type == null || type.length() == 0 || type == "?");
  }

  public boolean isTyped() {
    return !isUntyped();
  }

  public boolean isTextualAnonymous() {
    return name.startsWith("$");
  }

  public boolean isIncoming() {
    return direction.equals(Direction.IN);
  }

  @Override
  public Port clone() {
    Port p = new Port();
    p.setDirection(direction);
    p.setName(name);
    p.setType(type);
    p.setTypeReference(typeReference);
    p.setMappedPort(mappedPort);
    return p;
  }

  @Override
  public String toString() {
    return getFullName() + "|" + getComponent().getName() + "-" + getName();
  }

}
