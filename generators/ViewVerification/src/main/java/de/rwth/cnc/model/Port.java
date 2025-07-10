/* (c) https://github.com/MontiCore/monticore */
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
    if (getComponent() != null)
      return getFullName() + "|" + getComponent().getName() + "-" + getName();
    return getName();
  }

}
